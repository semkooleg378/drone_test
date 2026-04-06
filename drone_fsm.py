import math
import time
from enum import Enum

#from dronekit_sitl import start_default
from dronekit import connect, VehicleMode, LocationGlobalRelative

# =========================
# Налаштування задачі
# =========================

A_LAT = 50.450739
A_LON = 30.461242

B_LAT = 50.443326
B_LON = 30.448078

# Пороги
TARGET_ALT = 200.0
#TAKEOFF_DONE_ALT = 88.0
TAKEOFF_DONE_ALT = 200.0

APPROACH_DISTANCE_M = 20.0
LAND_DISTANCE_M = 5.0

MAX_HOLD_STEPS = 50

# Частота циклу
DT = 0.2  # 5 Гц

# RC-нейтралі
RC_ROLL_NEUTRAL = 1500
RC_PITCH_NEUTRAL = 1500
RC_THROTTLE_NEUTRAL = 1500
RC_YAW_NEUTRAL = 1500

# Обмеження команд
RC_MIN = 1100
RC_MAX = 1900

# Канали можуть відрізнятися у конкретній конфігурації, але зазвичай:
# 1 = roll, 2 = pitch, 3 = throttle, 4 = yaw
CH_ROLL = '1'
CH_PITCH = '2'
CH_THROTTLE = '3'
CH_YAW = '4'


class State(Enum):
    TAKEOFF = "TAKEOFF"
    HOLD = "HOLD"
    CAPTURE_LINE = "CAPTURE_LINE"
    CRUISE = "CRUISE"
    APPROACH = "APPROACH"
    LAND = "LAND"
    DONE = "DONE"
    FINAL_BRAKE = "FINAL_BRAKE"
    PRE_FINAL_ALIGN = "PRE_FINAL_ALIGN"

def set_param_checked(vehicle, name, value, retries=5, delay=1.0):

    for i in range(retries):
        try:
            print(f"set {name} = {value} (try {i+1})")
            vehicle.parameters[name] = value
            time.sleep(delay)

            cur = vehicle.parameters[name]
            print(f"readback {name} = {cur}")

            if cur is not None and abs(float(cur) - float(value)) < 1e-6:
                return True
        except Exception as e:
            print(f"set/read {name} failed: {e}")

        time.sleep(delay)

    return False
    
# =========================
# допоміжні clamp-функції
# =========================
def clampf(value, lo, hi):
    return max(lo, min(hi, float(value)))

def clamp_rc(value, lo=RC_MIN, hi=RC_MAX):
    return max(lo, min(hi, int(round(value))))
    
def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def distance_m(lat1, lon1, lat2, lon2):
    # Для невеликих відстаней достатньо
    dlat = (lat2 - lat1) * 111320.0
    dlon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


def bearing_deg(lat1, lon1, lat2, lon2):
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    dlonr = math.radians(lon2 - lon1)

    y = math.sin(dlonr) * math.cos(lat2r)
    x = (
        math.cos(lat1r) * math.sin(lat2r)
        - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlonr)
    )
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0

"""
def velocity_track_deg(vx, vy):
    # NED: vx = north, vy = east
    if math.hypot(vx, vy) < 0.3:
        return None
    track = math.degrees(math.atan2(vy, vx))
    return (track + 360.0) % 360.0
"""
EARTH_RADIUS_M = 6371000.0

def offset_point_m(lat_deg, lon_deg, bearing_deg_value, distance_m_value):
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)
    brng = math.radians(bearing_deg_value)
    d_div_r = distance_m_value / EARTH_RADIUS_M

    lat2 = math.asin(
        math.sin(lat1) * math.cos(d_div_r) +
        math.cos(lat1) * math.sin(d_div_r) * math.cos(brng)
    )

    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(d_div_r) * math.cos(lat1),
        math.cos(d_div_r) - math.sin(lat1) * math.sin(lat2)
    )

    return math.degrees(lat2), math.degrees(lon2)

def offset_gps(lat, lon, bearing_deg_value, dist_m):
    import math

    R = 6378137.0
    br = math.radians(bearing_deg_value)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(dist_m / R) +
        math.cos(lat1) * math.sin(dist_m / R) * math.cos(br)
    )

    lon2 = lon1 + math.atan2(
        math.sin(br) * math.sin(dist_m / R) * math.cos(lat1),
        math.cos(dist_m / R) - math.sin(lat1) * math.sin(lat2)
    )

    return math.degrees(lat2), math.degrees(lon2)
    
def velocity_track_deg(vx, vy):
    import math
    speed = math.hypot(vx, vy)
    if speed < 0.3:
        return None
    track = math.degrees(math.atan2(vy, vx))
    return (track + 360.0) % 360.0
    
def apply_deadband(x, db):
    if abs(x) <= db:
        return 0.0
    return x - db if x > 0 else x + db

def compute_track_error(prev_lat, prev_lon, lat, lon, target_lat, target_lon, last_track_error=None):
    move_dist = distance_m(prev_lat, prev_lon, lat, lon)
    desired_track = bearing_deg(lat, lon, target_lat, target_lon)

    # якщо майже не рухались — не довіряємо actual_track
    if move_dist < 1.0:
        return last_track_error if last_track_error is not None else 0.0

    actual_track = bearing_deg(prev_lat, prev_lon, lat, lon)
    return normalize_angle_deg(desired_track - actual_track)
    
class DroneFSM:
    def __init__(self, vehicle):
        self.DT = DT
        
        self.vehicle = vehicle
        self.state = State.TAKEOFF
        
        self.hold_counter = 0

        self.target_lat = B_LAT
        self.target_lon = B_LON
        self.target_alt = TARGET_ALT

        self.hold_start = None
        self.start_lat = None
        self.start_lon = None
        
        self.cruise_start_lat = None
        self.cruise_start_lon = None
        
        self.cruise_phase = "FAR"
        self.cruise_mode = "TRACK"
        self.cruise_kick_ticks = 0        
        
        #self.alt_first = None
        #self.alt_last = None
        #self.hold_counter = None
        self.prev_alt = None
        self.stable_hold_ticks = 0
        self.hover_throttle = 1470 #1360
        
        # для посадки
        self.prev_lat = 0.0 
        self.prev_lon = 0.0 
        self.cur_lat = 0.0 
        self.cur_lon = 0.0
        self.last_track_error = 0.0
        
        self.takeoff_start_lat = 0.0
        self.takeoff_start_lon = 0.0
        self.takeoff_start_time = None

        self.wind_est_speed = None
        self.wind_est_dir = None   
        
        self.approach_A_lat = None
        self.approach_A_lon = None
        self.use_approach_A = False        
        
        self.crab_bias = 0.0
        
        self.turn_stuck_ticks = 0
        self.last_heading = None
        
        self.prev_dist_to_target = None
        self.prev_track_error_abs = None
        self.no_progress_ticks = 0
        self.last_heading = None
        self.last_track_error = 0.0
        self.pos_hist = []      

        self.return_cooldown = 0        
        
        self.use_upwind_point = False
        self.upwind_lat = None
        self.upwind_lon = None
        
        self.capture_line_bearing = None
        self.capture_cross_sign = 0.0
        self.capture_target_heading = None
        self.capture_ok_ticks = 0
        self.capture_line_heading = None
        
        self.prev_capture_abs_cross = None
        self.capture_no_progress_ticks = 0
        
        self.approach_phase = None
        self.approach_target_alt = None
        
        self.land_offset_lat = None
        self.land_offset_lon = None
        
        self.approach_recap_latched = False
        self.approach_min_target_alt = None
        self.land_heading_ref = None        
        self.prev_nav_lat = None
        self.prev_nav_lon = None
        
        self.land_axis_heading = None
        self.prev_cross_line_err = None
                
    def read_state(self):
        pos = self.vehicle.location.global_relative_frame
        heading = self.vehicle.heading or 0

        return {
            "lat": pos.lat,
            "lon": pos.lon,
            "alt": pos.alt or 0.0,
            "heading": heading,
            "dist_to_target": distance_m(pos.lat, pos.lon, self.target_lat, self.target_lon),
            "bearing_to_target": bearing_deg(pos.lat, pos.lon, self.target_lat, self.target_lon),
            "dist_from_start": distance_m(pos.lat, pos.lon, self.start_lat, self.start_lon),
        }

    def set_rc(self, roll=None, pitch=None, throttle=None, yaw=None):
        overrides = {}

        if roll is not None:
            overrides[CH_ROLL] = clamp_rc(roll, RC_MIN, RC_MAX)
        if pitch is not None:
            overrides[CH_PITCH] = clamp_rc(pitch, RC_MIN, RC_MAX)
        if throttle is not None:
            overrides[CH_THROTTLE] = clamp_rc(throttle, RC_MIN, RC_MAX)
        if yaw is not None:
            overrides[CH_YAW] = clamp_rc(yaw, RC_MIN, RC_MAX)

        self.vehicle.channels.overrides = overrides

    def clear_rc(self):
        self.vehicle.channels.overrides = {}

    def wait_until_armable(self):
        while not self.vehicle.is_armable:
            print("Чекаємо готовність автопілота...")
            time.sleep(1)

    def arm(self):
        self.vehicle.mode = VehicleMode("STABILIZE")
        self.vehicle.armed = True

        start = time.time()
        while not self.vehicle.armed:
            if time.time() - start > 20:
                raise TimeoutError("Арминг не подтвердился")
            print("Чекаємо arm...")
            time.sleep(1)

    def initialize(self):
        self.wait_until_armable()
        self.arm()

        pos = self.vehicle.location.global_relative_frame
        self.start_lat = pos.lat
        self.start_lon = pos.lon

        print(f"Стартова точка: {self.start_lat}, {self.start_lon}")
        print("FSM ініціалізовано")

    def offset_gps(self, lat, lon, bearing_deg_value, dist_m):
        import math

        R = 6378137.0
        br = math.radians(bearing_deg_value)
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)

        lat2 = math.asin(
            math.sin(lat1) * math.cos(dist_m / R) +
            math.cos(lat1) * math.sin(dist_m / R) * math.cos(br)
        )

        lon2 = lon1 + math.atan2(
            math.sin(br) * math.sin(dist_m / R) * math.cos(lat1),
            math.cos(dist_m / R) - math.sin(lat1) * math.sin(lat2)
        )

        return math.degrees(lat2), math.degrees(lon2)
    
    def compute_upwind_point(self):
        if self.wind_est_dir is None:
            self.use_upwind_point = False
            self.upwind_lat = None
            self.upwind_lon = None
            print("[UPWIND] no wind estimate")
            return

        # якщо wind_est_dir = куди зносить, то проти вітру = +180
        wind_from = (self.wind_est_dir + 180.0) % 360.0

        UPWIND_DIST = 250.0

        self.upwind_lat, self.upwind_lon = self.offset_gps(B_LAT, B_LON, wind_from, UPWIND_DIST)
        self.use_upwind_point = True

        print(
            f"[UPWIND] wind_to={self.wind_est_dir:.1f} "
            f"wind_from={wind_from:.1f} "
            f"UP=({self.upwind_lat:.6f}, {self.upwind_lon:.6f}) "
            f"dist={UPWIND_DIST:.1f}"
        )

    def prepare_capture_line(self):

        def to_local_m(lat0, lon0, lat1, lon1):
            dx = (lon1 - lon0) * 111320.0 * math.cos(math.radians(lat0))
            dy = (lat1 - lat0) * 111320.0
            return dx, dy

        if self.wind_est_dir is None:
            self.capture_line_bearing = None
            self.capture_cross_sign = 0.0
            self.capture_target_heading = None
            print("[CAPTURE_LINE] no wind estimate")
            return

        # Лінія через B вздовж вітру.
        # Якщо wind_est_dir = куди зносить, то вісь лінії така сама.
        line_bearing = self.wind_est_dir

        # Друга точка на лінії через B
        ref2_lat, ref2_lon = self.offset_gps(B_LAT, B_LON, line_bearing, 100.0)

        # Поточне положення
        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon

        abx, aby = to_local_m(B_LAT, B_LON, ref2_lat, ref2_lon)
        apx, apy = to_local_m(B_LAT, B_LON, lat, lon)

        ab_len = math.hypot(abx, aby)
        if ab_len < 1e-6:
            self.capture_line_bearing = None
            self.capture_cross_sign = 0.0
            self.capture_target_heading = None
            print("[CAPTURE_LINE] invalid line")
            return

        # signed cross-track:
        # >0 з одного боку лінії, <0 з іншого
        cross = (apx * aby - apy * abx) / ab_len

        self.capture_line_bearing = normalize_angle_deg(line_bearing)
        self.capture_line_heading = self.capture_line_bearing
        self.capture_cross_sign = 1.0 if cross >= 0 else -1.0

        # Захоплюємо лінію поперечним рухом:
        # якщо ми праворуч від лінії, йдемо вліво на 90°
        # якщо ліворуч — вправо на 90°
        if self.capture_cross_sign > 0:
            self.capture_target_heading = normalize_angle_deg(self.capture_line_bearing + 90.0)
        else:
            self.capture_target_heading = normalize_angle_deg(self.capture_line_bearing - 90.0)
        print(
            f"[CAPTURE_LINE] line={self.capture_line_bearing:.1f} "
            f"cross_sign={self.capture_cross_sign:+.0f} "
            f"target_hdg={self.capture_target_heading:.1f}"
        )    
        
    def offset_gps(self, lat, lon, bearing_deg_value, dist_m):
        import math

        R = 6378137.0
        br = math.radians(bearing_deg_value)
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)

        lat2 = math.asin(
            math.sin(lat1) * math.cos(dist_m / R) +
            math.cos(lat1) * math.sin(dist_m / R) * math.cos(br)
        )

        lon2 = lon1 + math.atan2(
            math.sin(br) * math.sin(dist_m / R) * math.cos(lat1),
            math.cos(dist_m / R) - math.sin(lat1) * math.sin(lat2)
        )

        return math.degrees(lat2), math.degrees(lon2)

    def handle_takeoff(self, s):
        roll = RC_ROLL_NEUTRAL
        pitch = RC_PITCH_NEUTRAL
        yaw = RC_YAW_NEUTRAL

        if self.takeoff_start_time is None:
            self.takeoff_start_lat = s['lat']
            self.takeoff_start_lon = s['lon']
            self.takeoff_start_time = time.time()
            
        KD_ALT = 4.0

        if s['alt'] < TARGET_ALT * 0.75:
            throttle = 1580
        elif s['alt'] < TARGET_ALT * 0.88:
            throttle = 1565
        elif s['alt'] < TARGET_ALT * 0.95:
            throttle = 1545
        else:
            vertical_speed = (s['alt'] - self.prev_alt) / self.DT if self.prev_alt is not None else 0.0
            self.prev_alt = s['alt']

            base_takeoff_throttle = 1485
            throttle = base_takeoff_throttle - 3.0 * vertical_speed
            throttle = clamp_rc(throttle, 1450, 1500)    
            
        print(f"[TAKEOFF] alt={s['alt']:.2f} throttle={throttle}")

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        if s['alt'] >= TAKEOFF_DONE_ALT-0.5:
            self.state = State.HOLD
            self.hold_start = time.time()
            self.prev_alt = None
            self.stable_hold_ticks = 0
            self.hover_throttle = 1474 #1364 #1360
            self.hold_counter = 0
            print("Перехід TAKEOFF -> HOLD")

    def handle_hold(self, s):
        # Проста регуляція висоти
        error = TARGET_ALT - s['alt']
        
        roll = RC_ROLL_NEUTRAL
        pitch = RC_PITCH_NEUTRAL
        yaw = RC_YAW_NEUTRAL

        if self.wind_est_dir is None:
            drift_dist = distance_m(
                self.takeoff_start_lat, self.takeoff_start_lon,
                s['lat'], s['lon']
            )

            drift_dir = bearing_deg(
                self.takeoff_start_lat, self.takeoff_start_lon,
                s['lat'], s['lon']
            )

            drift_time = time.time() - self.takeoff_start_time

            if drift_time > 0.1:
                self.wind_est_speed = drift_dist / drift_time
                self.wind_est_dir = drift_dir
                #self.build_approach_point()
            print(
                f"[WIND_EST] dist={drift_dist:.1f}m "
                f"time={drift_time:.1f}s "
                f"dir={drift_dir:.1f} "
                f"speed={self.wind_est_speed:.2f}"
            )
                
        #HOVER_THROTTLE = 1360
        #1470
        KP_ALT = 2.5
        KD_ALT = 8.0

        vertical_speed = (s['alt'] - self.prev_alt) / self.DT if self.prev_alt is not None else 0.0
        self.prev_alt = s['alt']

        # Коефіцієнт можна підбирати
        # throttle = RC_THROTTLE_NEUTRAL + error * 2.5
        #throttle = 1440 + error * 2.5
        #throttle = self.vehicle.channels.overrides[CH_THROTTLE] + error * 2.5
        #throttle = HOVER_THROTTLE + KP_ALT * error - KD_ALT * vertical_speed
        throttle = self.hover_throttle + KP_ALT * error - KD_ALT * vertical_speed
        #throttle = self.vehicle.channels.overrides[CH_THROTTLE] + KP_ALT * error - KD_ALT * vertical_speed

        # Обмеження діапазону, щоб не розгойдувало
        # throttle = clamp(throttle, 1470, 1535)
        throttle = clamp_rc(throttle, 1300, 1525)

        #print(f"[HOLD] alt={s['alt']:.2f} error={error:.2f} throttle={throttle}")
        print(
                f"[HOLD] alt={s['alt']:.2f} "
                f"error={error:.2f} "
                f"vs={vertical_speed:.2f} "
                f"throttle={throttle}"
        )
        self.set_rc(
            #self.vehicle,
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )
        
        """
        if self.hold_counter >= 10:
            self.alt_first = self.alt_last
            self.alt_last = s['alt']
            self.hold_counter = 0
        self.hold_counter += 1
        """    
        # Для тесту тримаємо 15 секунд і виходимо
        """
        if self.hold_start is not None and time.time() - self.hold_start > 15 and math.fabs(TARGET_ALT-self.alt_first)+math.fabs(TARGET_ALT-self.alt_last) < 1.0:
            print("Тест утримання завершено")
            self.state = State.CRUISE
            self.cruise_start_lat = self.vehicle.location.global_relative_frame.lat
            self.cruise_start_lon = self.vehicle.location.global_relative_frame.lon
        """
        # Умова "заспокоїлись біля цілі"
        stable_alt = abs(error) < 1.0
        stable_vs = abs(vertical_speed) < 0.3

        if stable_alt and stable_vs:
            self.stable_hold_ticks += 1
        else:
            self.stable_hold_ticks = 0

        #if self.hold_start is not None and time.time() - self.hold_start > 10 and stable_alt and stable_vs:
        if self.hold_counter>=MAX_HOLD_STEPS or( self.hold_start is not None and time.time() - self.hold_start > 4 and self.stable_hold_ticks >= 4):
            print("Тест утримання завершено")
            self.prepare_capture_line()
            self.state = State.CAPTURE_LINE
            print("Перехід HOLD -> CAPTURE_LINE")
            self.cruise_start_lat = self.vehicle.location.global_relative_frame.lat
            self.cruise_start_lon = self.vehicle.location.global_relative_frame.lon
            self.prev_alt = None
            self.prev_capture_abs_cross = None
            self.capture_no_progress_ticks = 0
        self.hold_counter += 1

    def handle_capture_line(self, s):
        import math

        def to_local_m(lat0, lon0, lat1, lon1):
            dx = (lon1 - lon0) * 111320.0 * math.cos(math.radians(lat0))
            dy = (lat1 - lat0) * 111320.0
            return dx, dy

        vx, vy, vz = self.vehicle.velocity

        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        if self.wind_est_dir is None:
            print("[CAPTURE_LINE] no wind estimate")
            return

        # ---------------------------------------------------------
        # Лінія через B уздовж вітру
        # ---------------------------------------------------------
        line_bearing = self.wind_est_dir % 360.0
        ref2_lat, ref2_lon = self.offset_gps(B_LAT, B_LON, line_bearing, 100.0)

        abx, aby = to_local_m(B_LAT, B_LON, ref2_lat, ref2_lon)
        apx, apy = to_local_m(B_LAT, B_LON, lat, lon)

        ab_len = math.hypot(abx, aby)
        if ab_len < 1e-6:
            print("[CAPTURE_LINE] invalid geometry")
            return

        # signed cross-track
        cross_track = (apx * aby - apy * abx) / ab_len
        abs_cross = abs(cross_track)

        # вісь лінії
        axis = math.radians(line_bearing)

        # швидкості в координатах лінії
        cross_speed = vx * math.cos(axis) - vy * math.sin(axis)
        along_speed = vx * math.sin(axis) + vy * math.cos(axis)
        ground_speed = math.hypot(vx, vy)

        dist_to_B = distance_m(lat, lon, B_LAT, B_LON)

        # ---------------------------------------------------------
        # Прогрес
        # ---------------------------------------------------------
        if self.prev_capture_abs_cross is not None:
            if abs_cross < self.prev_capture_abs_cross - 1.0:
                self.capture_no_progress_ticks = 0
            else:
                self.capture_no_progress_ticks += 1
        else:
            self.capture_no_progress_ticks = 0

        self.prev_capture_abs_cross = abs_cross

        # ---------------------------------------------------------
        # Режими лише за близькістю до лінії
        # HARD - основний грубий переріз
        # FINAL - лише коли вже справді поруч
        # ---------------------------------------------------------
        if abs_cross > 70.0:
            mode = "HARD"
        else:
            mode = "FINAL"

        # ---------------------------------------------------------
        # Цільова поперечна швидкість
        # Для cross_track < 0 хочемо додатний cross_speed
        # Для cross_track > 0 хочемо від’ємний cross_speed
        # ---------------------------------------------------------
        if cross_track < 0.0:
            cut_heading = normalize_angle_deg(line_bearing + 90.0)
            if mode == "HARD":
                desired_cross_speed = 4.6
            else:
                desired_cross_speed = 1.8
        else:
            cut_heading = normalize_angle_deg(line_bearing - 90.0)
            if mode == "HARD":
                desired_cross_speed = -4.6
            else:
                desired_cross_speed = -1.8

        cross_speed_err = desired_cross_speed - cross_speed

        # ---------------------------------------------------------
        # target_heading
        #
        # Поки далеко - майже чистий переріз упоперек лінії.
        # Жодного nearest-point і жодного "летимо на B".
        # Лише невеликий bias за помилкою поперечної швидкості.
        # ---------------------------------------------------------
        if mode == "HARD":
            heading_bias = clampf(2.6 * cross_speed_err, -9.0, 9.0)
        else:
            heading_bias = clampf(1.8 * cross_speed_err, -6.0, 6.0)

        target_heading = normalize_angle_deg(cut_heading + heading_bias)
        heading_error = normalize_angle_deg(target_heading - heading)

        # ---------------------------------------------------------
        # Pitch
        #
        # Далеко: не душимо, даємо тягнути.
        # Але якщо прогрес млявий - трохи додаємо.
        # ---------------------------------------------------------
        if mode == "HARD":
            if abs_cross > 260.0:
                pitch_cmd = 1408
            elif abs_cross > 180.0:
                pitch_cmd = 1410
            elif abs_cross > 120.0:
                pitch_cmd = 1412
            else:
                pitch_cmd = 1414

            if self.capture_no_progress_ticks > 8:
                pitch_cmd -= 1
            if self.capture_no_progress_ticks > 16:
                pitch_cmd -= 1
            if self.capture_no_progress_ticks > 28:
                pitch_cmd -= 1

            if ground_speed < 3.0:
                pitch_cmd -= 1
            if ground_speed < 2.5:
                pitch_cmd -= 1

            pitch = clamp_rc(pitch_cmd, 1404, 1416)

        else:
            pitch_cmd = 1438
            if abs_cross < 45.0:
                pitch_cmd = 1442
            if abs_cross < 25.0:
                pitch_cmd = 1446

            # якщо ще надто швидко ріжемо лінію - трохи гальмуємо
            if abs(cross_speed) > 2.2:
                pitch_cmd += 1

            pitch = clamp_rc(pitch_cmd, 1436, 1448)

        # ---------------------------------------------------------
        # Roll
        #
        # Головний канал - ріжемо поперечну помилку.
        # Не лише heading_error, а й cross_speed_err.
        # ---------------------------------------------------------
        if mode == "HARD":
            roll_cmd = (
                1500
                + 2.0 * heading_error
                + 9.5 * cross_speed_err
            )
            roll = clamp_rc(roll_cmd, 1448, 1558)
        else:
            roll_cmd = (
                1500
                + 2.4 * heading_error
                + 6.0 * cross_speed_err
            )
            roll = clamp_rc(roll_cmd, 1455, 1548)

        # ---------------------------------------------------------
        # Yaw - допоміжний
        # ---------------------------------------------------------
        if mode == "HARD":
            yaw_cmd = 1500 + 3.4 * heading_error
            yaw = clamp_rc(yaw_cmd, 1455, 1545)
        else:
            yaw_cmd = 1500 + 4.0 * heading_error
            yaw = clamp_rc(yaw_cmd, 1460, 1540)

        # ---------------------------------------------------------
        # Вертикаль - тримаємо TARGET_ALT
        # ---------------------------------------------------------
        vertical_speed = (alt - self.prev_alt) / self.DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        KP_ALT = 2.5
        KD_ALT = 8.0
        throttle = self.hover_throttle + KP_ALT * (TARGET_ALT - alt) - KD_ALT * vertical_speed
        throttle = clamp_rc(throttle, 1000, 1525)

        # ---------------------------------------------------------
        # Команда
        # ---------------------------------------------------------
        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        print(
            f"[CAPTURE_LINE/V3/{mode}] "
            f"xtrack={cross_track:.1f} absx={abs_cross:.1f} "
            f"xs={cross_speed:.2f}/{desired_cross_speed:.2f} "
            f"as={along_speed:.2f} gs={ground_speed:.2f} "
            f"toB={dist_to_B:.1f} alt={alt:.1f} "
            f"hdg={heading:.1f} thdg={target_heading:.1f} herr={heading_error:.1f} "
            f"roll={roll} pitch={pitch} yaw={yaw} thr={throttle} "
            f"nprog={self.capture_no_progress_ticks}"
        )

        # ---------------------------------------------------------
        # Вихід у CRUISE
        #
        # Лише коли справді підійшли до лінії.
        # Жодних таймерів. Жодних forced exit.
        # ---------------------------------------------------------
        CAPTURE_X_OK = 10.0
        CAPTURE_X_GOOD = 6.0
        CAPTURE_CONFIRM_TICKS = 4

        if mode == "FINAL":
            if abs_cross <= 8.0 and abs(cross_speed) <= 1.2:
                self.capture_ok_ticks += 1
            #elif abs_cross <= CAPTURE_X_OK and abs(cross_speed) <= 2.2:
            #    self.capture_ok_ticks += 1
            else:
                self.capture_ok_ticks = 0
        else:
            self.capture_ok_ticks = 0

        if self.capture_ok_ticks >= CAPTURE_CONFIRM_TICKS:
            self.capture_line_heading = line_bearing
            print(
                f"[CAPTURE_LINE] line captured -> CRUISE "
                f"(absx={abs_cross:.1f}, xs={cross_speed:.2f}, toB={dist_to_B:.1f})"
            )
            self.capture_ok_ticks = 0
            self.cruise_phase = "FAR"
            self.cruise_mode = "TRACK"
            self.cruise_kick_ticks = 0
            self.state = State.CRUISE
            return
            
    def handle_cruise_along_line(self, s):
        import math

        def to_local_m(lat0, lon0, lat1, lon1):
            dx = (lon1 - lon0) * 111320.0 * math.cos(math.radians(lat0))
            dy = (lat1 - lat0) * 111320.0
            return dx, dy

        vx, vy, vz = self.vehicle.velocity

        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        if self.capture_line_heading is not None:
            line_bearing = self.capture_line_heading % 360.0
        elif self.wind_est_dir is not None:
            line_bearing = self.wind_est_dir % 360.0
        else:
            line_bearing = bearing_deg(lat, lon, B_LAT, B_LON)

        ref2_lat, ref2_lon = self.offset_gps(B_LAT, B_LON, line_bearing, 100.0)

        abx, aby = to_local_m(B_LAT, B_LON, ref2_lat, ref2_lon)
        apx, apy = to_local_m(B_LAT, B_LON, lat, lon)

        ab_len = math.hypot(abx, aby)
        if ab_len < 1e-6:
            print("[CRUISE_ALONG_LINE] invalid geometry")
            return

        cross_line_err = (apx * aby - apy * abx) / ab_len
        along_line_err = (apx * abx + apy * aby) / ab_len

        axis = math.radians(line_bearing)
        cross_line_speed = vx * math.cos(axis) - vy * math.sin(axis)
        along_line_speed = vx * math.sin(axis) + vy * math.cos(axis)

        dist_to_B = distance_m(lat, lon, B_LAT, B_LON)
        bearing_to_B = bearing_deg(lat, lon, B_LAT, B_LON)

        br = math.radians(bearing_to_B)
        cls = vx * math.cos(br) + vy * math.sin(br)
        latv = -vx * math.sin(br) + vy * math.cos(br)
        ground_speed = math.hypot(vx, vy)

        abs_cross = abs(cross_line_err)
        abs_along = abs(along_line_err)
        drift_angle = normalize_angle_deg(bearing_to_B - heading)

        if not hasattr(self, "cruise_phase"):
            self.cruise_phase = "FAR"
        if not hasattr(self, "cruise_mode"):
            self.cruise_mode = "TRACK"
        if not hasattr(self, "cruise_kick_ticks"):
            self.cruise_kick_ticks = 0

        # ---------------------------------------------------------
        # FAR: TRACK + KICKBACK
        # ---------------------------------------------------------
        if self.cruise_phase == "FAR":
            if self.cruise_mode == "KICKBACK":
                self.cruise_kick_ticks -= 1
                if self.cruise_kick_ticks <= 0:
                    self.cruise_mode = "TRACK"
            else:
                if abs_cross > 8.2 or (abs_cross > 7.6 and abs(cross_line_speed) > 1.25):
                    self.cruise_mode = "KICKBACK"
                    self.cruise_kick_ticks = 5

        # ---------------------------------------------------------
        # FAR -> NEAR раніше
        # ---------------------------------------------------------
        go_near = (
            self.cruise_phase == "FAR" and
            (
                (dist_to_B <= 540.0 and abs_cross <= 10.5 and abs(cross_line_speed) <= 1.40) or
                (dist_to_B <= 500.0)
            )
        )

        if go_near:
            self.cruise_phase = "NEAR"
            self.cruise_mode = "TRACK"
            self.cruise_kick_ticks = 0
            print(
                f"[CRUISE_ALONG_LINE] FAR -> NEAR "
                f"toB={dist_to_B:.1f} crossE={cross_line_err:.1f} crossS={cross_line_speed:.2f} "
                f"alongE={along_line_err:.1f} alongS={along_line_speed:.2f} "
                f"alt={alt:.1f} cls={cls:.2f} latv={latv:.2f}"
            )

        # ---------------------------------------------------------
        # Межі фаз
        # ---------------------------------------------------------
        if self.cruise_phase == "FAR":
            roll_min, roll_max = 1428, 1572
            pitch_min, pitch_max = 1470, 1481
        else:
            roll_min, roll_max = 1445, 1558
            pitch_min, pitch_max = 1468, 1480

        # ---------------------------------------------------------
        # Висота
        # FAR — як було
        # NEAR — нижчий descent corridor
        # ---------------------------------------------------------
        if self.cruise_phase == "FAR":
            if dist_to_B > 350.0:
                target_alt = 200.0
            elif dist_to_B > 310.0:
                target_alt = 170.0
            elif dist_to_B > 270.0:
                target_alt = 135.0
            elif dist_to_B > 230.0:
                target_alt = 100.0
            elif dist_to_B > 190.0:
                target_alt = 72.0
            elif dist_to_B > 155.0:
                target_alt = 50.0
            elif dist_to_B > 125.0:
                target_alt = 34.0
            elif dist_to_B > 95.0:
                target_alt = 22.0
            else:
                target_alt = 14.0

            target_alt = max(target_alt, alt - 10.0)

        else:
            # Нижчий NEAR corridor, щоб не зависати на ~120 м
            if dist_to_B > 260.0:
                target_alt = 95.0
            elif dist_to_B > 210.0:
                target_alt = 70.0
            elif dist_to_B > 170.0:
                target_alt = 50.0
            elif dist_to_B > 130.0:
                target_alt = 34.0
            elif dist_to_B > 95.0:
                target_alt = 22.0
            else:
                target_alt = 14.0

            # У NEAR ще сильніше тягнемо вниз
            target_alt = max(target_alt, alt - 14.0)

            # Якщо вісь попливла — трохи притримуємо, але не даємо знову висіти високо
            if abs_cross > 10.0:
                target_alt = max(target_alt, alt - 8.0)
            elif abs_cross > 9.0:
                target_alt = max(target_alt, alt - 10.0)

        # ---------------------------------------------------------
        # Керування по лінії
        # ---------------------------------------------------------
        if self.cruise_phase == "FAR":
            if self.cruise_mode == "TRACK":
                desired_along_speed = 3.0
                target_heading = normalize_angle_deg(
                    line_bearing
                    + clampf(-1.3 * cross_line_err, -12.0, 12.0)
                    + clampf(-2.2 * cross_line_speed, -9.0, 9.0)
                )
                roll_cmd = (
                    1500
                    + 1.35 * normalize_angle_deg(target_heading - heading)
                    + 1.8 * (-latv)
                    + 1.3 * (-cross_line_err)
                    + 4.2 * (-cross_line_speed)
                )
            else:
                desired_along_speed = 2.4
                target_heading = normalize_angle_deg(
                    line_bearing
                    + clampf(-3.0 * cross_line_err, -22.0, 22.0)
                    + clampf(-4.8 * cross_line_speed, -14.0, 14.0)
                )
                roll_cmd = (
                    1500
                    + 1.15 * normalize_angle_deg(target_heading - heading)
                    + 2.6 * (-cross_line_err)
                    + 6.2 * (-cross_line_speed)
                    + 1.8 * (-latv)
                )
        else:
            # Логіку лінії майже не змінюємо
            desired_along_speed = 1.35
            target_heading = normalize_angle_deg(
                line_bearing
                + clampf(-2.8 * cross_line_err, -16.0, 16.0)
                + clampf(-3.4 * cross_line_speed, -10.0, 10.0)
                + clampf(0.20 * normalize_angle_deg(bearing_to_B - line_bearing), -5.0, 5.0)
            )
            roll_cmd = (
                1500
                + 1.5 * normalize_angle_deg(target_heading - heading)
                + 2.2 * (-latv)
                + 2.8 * (-cross_line_err)
                + 5.4 * (-cross_line_speed)
            )

        heading_error = normalize_angle_deg(target_heading - heading)
        roll = clamp_rc(roll_cmd, roll_min, roll_max)

        # ---------------------------------------------------------
        # Pitch
        # ---------------------------------------------------------
        along_err = desired_along_speed - along_line_speed
        pitch_cmd = 1472 - 8.0 * along_err

        if self.cruise_phase == "FAR":
            if self.cruise_mode == "KICKBACK":
                pitch_cmd += 1
                if abs_cross > 9.0:
                    pitch_cmd += 1
                if abs_cross > 10.5:
                    pitch_cmd += 1
        else:
            if abs_cross > 7.5:
                pitch_cmd += 1
            if abs_cross > 9.0:
                pitch_cmd += 1
            if abs_cross > 10.0:
                pitch_cmd += 1
            if abs(cross_line_speed) > 1.05:
                pitch_cmd += 1
            if dist_to_B <= 220.0:
                pitch_cmd += 1
            if dist_to_B <= 170.0:
                pitch_cmd += 1

        pitch = clamp_rc(pitch_cmd, pitch_min, pitch_max)

        # ---------------------------------------------------------
        # Вертикаль
        # ---------------------------------------------------------
        vertical_speed = (alt - self.prev_alt) / self.DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        KP_ALT = 6.2 if self.cruise_phase == "FAR" else 9.2
        KD_ALT = 9.5
        throttle = self.hover_throttle + KP_ALT * (target_alt - alt) - KD_ALT * vertical_speed
        throttle = clamp_rc(throttle, 1080, 1525)

        # ---------------------------------------------------------
        # Yaw
        # ---------------------------------------------------------
        yaw_cmd = 1500 + 3.0 * heading_error
        yaw = clamp_rc(yaw_cmd, 1440, 1560)

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        print(
            f"[CRUISE_ALONG_LINE/{self.cruise_phase}/{self.cruise_mode}] "
            f"toB={dist_to_B:.1f} alt={alt:.1f}/{target_alt:.1f} "
            f"crossE={cross_line_err:.1f} crossS={cross_line_speed:.2f} "
            f"alongE={along_line_err:.1f} alongS={along_line_speed:.2f}/{desired_along_speed:.2f} "
            f"gs={ground_speed:.2f} cls={cls:.2f} latv={latv:.2f} drift={drift_angle:.1f} "
            f"hdg={heading:.1f} axis={line_bearing:.1f} B={bearing_to_B:.1f} "
            f"thdg={target_heading:.1f} herr={heading_error:.1f} "
            f"roll={roll} pitch={pitch} yaw={yaw} thr={throttle} "
            f"kick={self.cruise_kick_ticks}"
        )

        # ---------------------------------------------------------
        # NEAR -> APPROACH
        # ---------------------------------------------------------
        enter_approach = (
            self.cruise_phase == "NEAR" and
            along_line_err < 0.0 and
            abs_along <= 260.0 and
            abs_cross <= 8.8 and
            abs(cross_line_speed) <= 1.05 and
            alt <= 80.0 and
            cls >= 1.0
        )

        if enter_approach:
            self.approach_axis_heading = line_bearing
            self.approach_overshoot_ticks = 0
            print(
                f"[CRUISE_ALONG_LINE->APPROACH] "
                f"toB={dist_to_B:.1f} "
                f"crossE={cross_line_err:.1f} crossS={cross_line_speed:.2f} "
                f"alongE={along_line_err:.1f} alongS={along_line_speed:.2f} "
                f"alt={alt:.1f} cls={cls:.2f} latv={latv:.2f}"
            )
            self.state = State.APPROACH
            return            
            
    def handle_approach(self, s):
        import math

        def to_local_m(lat0, lon0, lat1, lon1):
            dx = (lon1 - lon0) * 111320.0 * math.cos(math.radians(lat0))
            dy = (lat1 - lat0) * 111320.0
            return dx, dy

        vx, vy, vz = self.vehicle.velocity

        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        dist_to_B = distance_m(lat, lon, B_LAT, B_LON)
        bearing_to_B = bearing_deg(lat, lon, B_LAT, B_LON)
        ground_speed = math.hypot(vx, vy)

        # ---------------------------------------------------------
        # Базова вісь заходу
        # ---------------------------------------------------------
        if getattr(self, "approach_axis_heading", None) is not None:
            axis_heading = self.approach_axis_heading % 360.0
        elif getattr(self, "capture_line_heading", None) is not None:
            axis_heading = self.capture_line_heading % 360.0
        elif self.wind_est_dir is not None:
            axis_heading = self.wind_est_dir % 360.0
        else:
            axis_heading = bearing_to_B

        axis = math.radians(axis_heading)
        br = math.radians(bearing_to_B)

        # ---------------------------------------------------------
        # Геометрія відносно осі через B
        # ---------------------------------------------------------
        ref2_lat, ref2_lon = self.offset_gps(B_LAT, B_LON, axis_heading, 100.0)
        abx, aby = to_local_m(B_LAT, B_LON, ref2_lat, ref2_lon)
        apx, apy = to_local_m(B_LAT, B_LON, lat, lon)

        ab_len = math.hypot(abx, aby)
        if ab_len < 1e-6:
            print("[APPROACH] invalid geometry")
            return

        cross_line_err = (apx * aby - apy * abx) / ab_len
        along_line_err = (apx * abx + apy * aby) / ab_len

        cross_line_speed = vx * math.cos(axis) - vy * math.sin(axis)
        along_line_speed = vx * math.sin(axis) + vy * math.cos(axis)

        # ---------------------------------------------------------
        # Швидкості відносно реальної B
        # ---------------------------------------------------------
        cls = vx * math.cos(br) + vy * math.sin(br)
        latv = -vx * math.sin(br) + vy * math.cos(br)

        abs_cross = abs(cross_line_err)
        abs_cross_speed = abs(cross_line_speed)
        abs_latv = abs(latv)

        # ---------------------------------------------------------
        # Overshoot / no-progress
        # ---------------------------------------------------------
        if not hasattr(self, "approach_overshoot_ticks"):
            self.approach_overshoot_ticks = 0

        overshoot_now = (
            (dist_to_B <= 28.0 and cls <= 0.35) or
            (dist_to_B <= 22.0 and along_line_err >= -2.0) or
            (dist_to_B <= 28.0 and abs_cross >= 16.0)
        )

        if overshoot_now:
            self.approach_overshoot_ticks += 1
        else:
            self.approach_overshoot_ticks = 0

        overshoot_active = self.approach_overshoot_ticks >= 1

        # ---------------------------------------------------------
        # Phase select
        # POINT лише якщо геометрія вже пристойна
        # ---------------------------------------------------------
        point_ready = (
            dist_to_B <= 10.0 and
            abs_cross <= 6.5 and
            abs_latv <= 0.9
        )

        force_point_on_overshoot = (
            self.approach_overshoot_ticks >= 2 and
            dist_to_B <= 16.0 and
            abs_cross <= 7.5 and
            abs_latv <= 0.9
        )

        if point_ready or force_point_on_overshoot:
            phase = "POINT"
        else:
            phase = "AXIS"

        # ---------------------------------------------------------
        # Close-range recapture
        # ---------------------------------------------------------
        close_axis_recapture = (
            phase == "AXIS" and
            dist_to_B <= 36.0 and
            abs_cross >= 8.5
        )

        # було надто рано / надто "рятувально"
        developing_miss = (
            phase == "AXIS" and
            dist_to_B <= 12.0 and
            abs_cross > 8.7
        )

        # Latch close recapture
        if not hasattr(self, "approach_recap_latched"):
            self.approach_recap_latched = False

        if phase == "AXIS":
            if dist_to_B <= 36.0 and abs_cross >= 8.5:
                self.approach_recap_latched = True
            elif self.approach_recap_latched:
                if dist_to_B > 40.0 or abs_cross <= 7.2:
                    self.approach_recap_latched = False
        else:
            self.approach_recap_latched = False

        close_axis_recapture = close_axis_recapture or self.approach_recap_latched

        # ---------------------------------------------------------
        # Heading
        # ---------------------------------------------------------
        if phase == "AXIS":
            if dist_to_B > 90.0:
                point_bias = 0.06
            elif dist_to_B > 60.0:
                point_bias = 0.10
            elif dist_to_B > 35.0:
                point_bias = 0.16
            else:
                point_bias = 0.20

            axis_to_B_err = normalize_angle_deg(bearing_to_B - axis_heading)
            base_heading = normalize_angle_deg(axis_heading + point_bias * axis_to_B_err)

            if close_axis_recapture:
                target_heading = normalize_angle_deg(
                    base_heading
                    + clampf(-5.2 * cross_line_err,  -44.0, 44.0)
                    + clampf(-6.4 * cross_line_speed, -28.0, 28.0)
                    + clampf(-3.2 * latv,            -24.0, 24.0)
                )
            elif abs_cross > 8.0 or abs_cross_speed > 0.8 or overshoot_active:
                target_heading = normalize_angle_deg(
                    base_heading
                    + clampf(-3.8 * cross_line_err,  -36.0, 36.0)
                    + clampf(-5.8 * cross_line_speed, -28.0, 28.0)
                    + clampf(-3.0 * latv,            -24.0, 24.0)
                )
            else:
                target_heading = normalize_angle_deg(
                    base_heading
                    + clampf(-3.0 * cross_line_err,  -28.0, 28.0)
                    + clampf(-4.4 * cross_line_speed, -22.0, 22.0)
                    + clampf(-2.4 * latv,            -18.0, 18.0)
                )
        else:
            target_heading = normalize_angle_deg(
                bearing_to_B
                + clampf(-3.0 * latv, -24.0, 24.0)
                + clampf(-0.8 * cross_line_err, -10.0, 10.0)
            )

        heading_error = normalize_angle_deg(target_heading - heading)

        # ---------------------------------------------------------
        # Висота
        # ---------------------------------------------------------
        if dist_to_B > 180.0:
            target_alt = 46.0
        elif dist_to_B > 130.0:
            target_alt = 34.0
        elif dist_to_B > 90.0:
            target_alt = 23.0
        elif dist_to_B > 60.0:
            target_alt = 14.0
        elif dist_to_B > 40.0:
            target_alt = 9.0
        elif dist_to_B > 28.0:
            target_alt = 6.0
        elif dist_to_B > 20.0:
            target_alt = 4.2
        elif dist_to_B > 15.0:
            target_alt = 3.0
        elif dist_to_B > 10.0:
            target_alt = 2.2
        else:
            target_alt = 1.6
        
        # Поки поперечка погана — не валимося вниз надто бадьоро,
        # але й не зависаємо високо у ближньому recapture
        if phase == "AXIS":
            if abs_cross > 10.0:
                target_alt = max(target_alt, alt - 1.1)
            if abs_cross > 14.0:
                target_alt = max(target_alt, alt - 0.6)
            if abs_cross > 18.0:
                target_alt = max(target_alt, alt - 0.2)

            if abs_latv > 1.0:
                target_alt = max(target_alt, alt - 0.5)
            if abs_latv > 1.4:
                target_alt = max(target_alt, alt - 0.1)

            if overshoot_active or close_axis_recapture:
                if dist_to_B <= 16.0:
                    target_alt = max(target_alt, alt - 1.20)
                elif dist_to_B <= 24.0:
                    target_alt = max(target_alt, alt - 0.90)
                else:
                    target_alt = max(target_alt, alt - 0.30)

            if developing_miss:
                target_alt = max(target_alt, alt - 0.70)
            
        vertical_speed = (alt - self.prev_alt) / self.DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        KP_ALT = 6.4
        KD_ALT = 11.2
        throttle = self.hover_throttle + KP_ALT * (target_alt - alt) - KD_ALT * vertical_speed
        throttle = clamp_rc(throttle, 1000, 1510)

        # ---------------------------------------------------------
        # Pitch
        # ---------------------------------------------------------
        if phase == "AXIS":
            if dist_to_B > 90.0:
                desired_cls = 1.20
            elif dist_to_B > 70.0:
                desired_cls = 1.08
            elif dist_to_B > 55.0:
                desired_cls = 0.96
            elif dist_to_B > 40.0:
                desired_cls = 0.82
            elif dist_to_B > 28.0:
                desired_cls = 0.68
            else:
                desired_cls = 0.52

            if abs_cross > 8.0 or abs_cross_speed > 0.8:
                desired_cls = max(desired_cls, 1.15)
            if abs_cross > 12.0:
                desired_cls = max(desired_cls, 1.25)
            if abs_cross > 16.0:
                desired_cls = max(desired_cls, 1.35)

            if overshoot_active:
                desired_cls = max(desired_cls, 1.15)
                if abs_cross > 16.0:
                    desired_cls = max(desired_cls, 1.30)

            # ближній recapture м’якший, без вічного зависання
            if close_axis_recapture:
                desired_cls = max(desired_cls, 0.92)
                if abs_cross > 9.3:
                    desired_cls = max(desired_cls, 1.00)
                if abs_cross > 10.8:
                    desired_cls = max(desired_cls, 1.10)

            if developing_miss:
                desired_cls = max(desired_cls, 1.12)

            # не даємо зовсім вбити поздовжню, поки cross ще поганий
            if dist_to_B <= 14.0 and abs_cross > 8.0:
                desired_cls = max(desired_cls, 0.72)
            if dist_to_B <= 11.0 and abs_cross > 8.0:
                desired_cls = max(desired_cls, 0.62)

            cls_err = desired_cls - cls
            pitch_cmd = 1547 - 6.2 * cls_err

            if cls > desired_cls + 1.50:
                pitch_cmd += 1
            if cls > desired_cls + 2.10:
                pitch_cmd += 1

            if cls < desired_cls - 0.45:
                pitch_cmd -= 1
            if cls < desired_cls - 0.90:
                pitch_cmd -= 1
            if cls < desired_cls - 1.30:
                pitch_cmd -= 1

            if abs_cross > 10.0:
                pitch_cmd += 1
            if abs_latv > 1.0:
                pitch_cmd += 1

            if overshoot_active:
                pitch_cmd = min(pitch_cmd, 1549)

            if close_axis_recapture:
                if dist_to_B <= 16.0:
                    pitch_cmd = min(pitch_cmd, 1544)
                else:
                    pitch_cmd = min(pitch_cmd, 1545)
                    
            if developing_miss:
                pitch_cmd = min(pitch_cmd, 1545)

            pitch = clamp_rc(pitch_cmd, 1540, 1551)

        else:
            if dist_to_B > 10.0:
                desired_cls = 0.32
            elif dist_to_B > 6.0:
                desired_cls = 0.18
            else:
                desired_cls = 0.07

            cls_err = desired_cls - cls
            pitch_cmd = 1550 - 17.0 * cls_err

            if self.approach_overshoot_ticks >= 2:
                pitch_cmd += 2

            pitch = clamp_rc(pitch_cmd, 1546, 1558)

        # ---------------------------------------------------------
        # Roll
        # ---------------------------------------------------------
        if phase == "AXIS":
            if close_axis_recapture:
                roll_cmd = (
                    1500
                    + 4.6 * heading_error
                    + 11.5 * (-cross_line_speed)
                    + 4.4 * (-latv)
                    + 3.8 * (-cross_line_err)
                )
                roll = clamp_rc(roll_cmd, 1450, 1570)
            elif abs_cross > 8.0 or abs_cross_speed > 0.8 or overshoot_active:
                roll_cmd = (
                    1500
                    + 4.2 * heading_error
                    + 11.0 * (-cross_line_speed)
                    + 4.4 * (-latv)
                    + 2.2 * (-cross_line_err)
                )
                roll = clamp_rc(roll_cmd, 1456, 1562)
            else:
                roll_cmd = (
                    1500
                    + 3.5 * heading_error
                    + 8.8 * (-cross_line_speed)
                    + 3.5 * (-latv)
                    + 1.7 * (-cross_line_err)
                )
                roll = clamp_rc(roll_cmd, 1460, 1558)
        else:
            roll_cmd = (
                1500
                + 3.4 * heading_error
                + 6.4 * (-latv)
                + 1.1 * (-cross_line_err)
            )
            roll = clamp_rc(roll_cmd, 1468, 1550)

        # ---------------------------------------------------------
        # Yaw
        # ---------------------------------------------------------
        yaw_cmd = 1500 + 4.0 * heading_error
        yaw = clamp_rc(yaw_cmd, 1458, 1542)

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        print(
            f"[APPROACH/{phase}] "
            f"toB={dist_to_B:.1f} alt={alt:.1f}/{target_alt:.1f} "
            f"crossE={cross_line_err:.1f} crossS={cross_line_speed:.2f} "
            f"alongE={along_line_err:.1f} alongS={along_line_speed:.2f} "
            f"cls={cls:.2f}/{desired_cls:.2f} latv={latv:.2f} "
            f"gs={ground_speed:.2f} "
            f"hdg={heading:.1f} axis={axis_heading:.1f} B={bearing_to_B:.1f} thdg={target_heading:.1f} "
            f"herr={heading_error:.1f} "
            f"roll={roll} pitch={pitch} yaw={yaw} thr={throttle} "
            f"ovr={self.approach_overshoot_ticks} "
            f"recap={1 if close_axis_recapture else 0} "
            f"miss={1 if developing_miss else 0}"
        )

        # ---------------------------------------------------------
        # Last-chance LAND gate:
        # якщо ми вже майже біля точки і approach далі краще не зробить,
        # входимо в LAND раніше, щоб не пролетіти повз
        # ---------------------------------------------------------
        last_chance_land = (
            dist_to_B <= 10.5 and
            alt <= 9.0 and
            abs_cross <= 8.4 and
            abs_latv <= 1.15 and
            cls <= 0.45 and
            (
                along_line_err >= -2.5 or
                self.approach_overshoot_ticks >= 2
            )
        )

        if last_chance_land:
            self.land_heading_ref = bearing_to_B
            self.land_axis_heading = axis_heading
            self.approach_recap_latched = False
            print(
                f"[APPROACH->LAND/LAST_CHANCE] "
                f"toB={dist_to_B:.1f} alt={alt:.1f} "
                f"cls={cls:.2f} latv={latv:.2f} "
                f"crossE={cross_line_err:.1f} alongE={along_line_err:.1f} "
                f"ovr={self.approach_overshoot_ticks}"
            )
            self.state = State.LAND
            return            
        # ---------------------------------------------------------
        # Перехід у LAND
        # Трохи м’якший gate, щоб не зависати в APPROACH,
        # коли геометрія вже майже зібрана
        # ---------------------------------------------------------
        if (
            dist_to_B <= 12.0 and
            alt <= 9.0 and
            cls <= 0.80 and
            abs_latv <= 1.05 and
            abs_cross <= 8.6
        ):
            self.land_heading_ref = bearing_to_B
            self.land_axis_heading = axis_heading
            self.approach_recap_latched = False
            print(
                f"[APPROACH->LAND] "
                f"toB={dist_to_B:.1f} alt={alt:.1f} "
                f"cls={cls:.2f} latv={latv:.2f} "
                f"crossE={cross_line_err:.1f}"
            )
            self.state = State.LAND
            return

            
    def handle_land(self, s):
        import math

        vx, vy, vz = self.vehicle.velocity

        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        dist_to_B = distance_m(lat, lon, B_LAT, B_LON)
        bearing_to_B = bearing_deg(lat, lon, B_LAT, B_LON)
        ground_speed = math.hypot(vx, vy)

        # ---------------------------------------------------------
        # Базові reference'и LAND
        # ---------------------------------------------------------
        axis_heading = getattr(self, "land_axis_heading", None)
        if axis_heading is None:
            axis_heading = getattr(self, "approach_axis_heading", None)
        if axis_heading is None:
            axis_heading = getattr(self, "capture_line_heading", None)
        if axis_heading is None:
            axis_heading = bearing_to_B
        axis_heading = axis_heading % 360.0

        heading_ref = getattr(self, "land_heading_ref", None)
        if heading_ref is None:
            heading_ref = bearing_to_B
        heading_ref = heading_ref % 360.0

        # ---------------------------------------------------------
        # Геометрія відносно осі через B
        # ---------------------------------------------------------
        def to_local_m(lat0, lon0, lat1, lon1):
            dx = (lon1 - lon0) * 111320.0 * math.cos(math.radians(lat0))
            dy = (lat1 - lat0) * 111320.0
            return dx, dy

        axis_ref_lat, axis_ref_lon = self.offset_gps(B_LAT, B_LON, axis_heading, 100.0)

        abx, aby = to_local_m(B_LAT, B_LON, axis_ref_lat, axis_ref_lon)
        apx, apy = to_local_m(B_LAT, B_LON, lat, lon)

        ab_len = math.hypot(abx, aby)
        if ab_len < 1e-6:
            print("[LAND] invalid geometry")
            return

        crossE = (apx * aby - apy * abx) / ab_len
        alongE = (apx * abx + apy * aby) / ab_len

        axis = math.radians(axis_heading)
        br = math.radians(bearing_to_B)

        alongS = vx * math.sin(axis) + vy * math.cos(axis)
        crossS = vx * math.cos(axis) - vy * math.sin(axis)

        cls = vx * math.cos(br) + vy * math.sin(br)
        latv = -vx * math.sin(br) + vy * math.cos(br)

        # ---------------------------------------------------------
        # Вертикальна швидкість
        # ---------------------------------------------------------
        vertical_speed = (alt - self.prev_alt) / self.DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        # ---------------------------------------------------------
        # Фази LAND
        # Трохи раніше у WIND_MODE
        # ---------------------------------------------------------
        if alt > 4.8:
            phase = "SMART_FLARE"
        else:
            phase = "WIND_MODE"
    
        # ---------------------------------------------------------
        # Yaw
        # ---------------------------------------------------------
        heading_error = normalize_angle_deg(axis_heading - heading)
        yaw_cmd = 1500 + 0.8 * heading_error
        yaw = clamp_rc(yaw_cmd, 1476, 1524)

        # ---------------------------------------------------------
        # SMART_FLARE
        # ---------------------------------------------------------
        if phase == "SMART_FLARE":
            # Ще трохи більш гальмівний flare
            if dist_to_B > 10.0:
                pitch = 1485
            elif dist_to_B > 8.0:
                pitch = 1483
            else:
                pitch = 1482

            if abs(latv) > 1.0 or abs(crossE) > 7.8:
                pitch = min(pitch, 1483)
            if abs(latv) > 1.5:
                pitch = min(pitch, 1482)
    
            # Трохи сильніший anti-drift + трохи ширші межі,
            # щоб не впиратися надто рано
            roll_cmd = (
                1500
                + 6.6 * (-latv)
                + 4.6 * (-crossE)
                + 4.8 * (-crossS)
            )
            roll = clamp_rc(roll_cmd, 1478, 1528)

            if alt > 6.2:
                target_vs = -1.35
            elif alt > 5.6:
                target_vs = -1.18
            else:
                target_vs = -1.00

            vs_err = target_vs - vertical_speed
            throttle = self.hover_throttle + 10.5 * vs_err
            throttle = clamp_rc(throttle, 1452, 1461)

            self.set_rc(
                roll=roll,
                pitch=pitch,
                throttle=throttle,
                yaw=yaw
            )

            print(
                f"[LAND/{phase}] "
                f"toB={dist_to_B:.1f} alt={alt:.1f} "
                f"gs={ground_speed:.2f} cls={cls:.2f} latv={latv:.2f} "
                f"alongE={alongE:.2f} crossE={abs(crossE):.2f} "
                f"alongS={alongS:.2f} crossS={crossS:.2f} "
                f"vs={vertical_speed:.2f} hdg={heading:.1f} axis={axis_heading:.1f} "
                f"roll={roll} pitch={pitch} yaw={yaw} thr={throttle}"
            )
            return

        # ---------------------------------------------------------
        # WIND_MODE
        # ---------------------------------------------------------
        # Ще трохи менш "протягувальний" pitch
        pitch = 1481
        if abs(latv) > 1.2:
            pitch = 1482
        if abs(latv) > 1.7:
            pitch = 1483

        # Трохи сильніший anti-drift
        roll_cmd = (
            1500
            + 5.8 * (-latv)
            + 4.2 * (-crossE)
            + 4.2 * (-crossS)
        )
        roll = clamp_rc(roll_cmd, 1482, 1522)

        if alt > 3.8:
            target_vs = -1.55
        elif alt > 2.6:
            target_vs = -1.35
        elif alt > 1.5:
            target_vs = -1.15
        else:
            target_vs = -0.90

        vs_err = target_vs - vertical_speed
        throttle = self.hover_throttle + 10.0 * vs_err

        if abs(latv) > 1.3 and alt < 3.5:
            throttle += 2

        throttle = clamp_rc(throttle, 1452, 1460)

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=1500
        )

        print(
            f"[LAND/{phase}] "
            f"toB={dist_to_B:.1f} alt={alt:.1f} "
            f"gs={ground_speed:.2f} cls={cls:.2f} latv={latv:.2f} "
            f"alongE={alongE:.2f} crossE={abs(crossE):.2f} "
            f"alongS={alongS:.2f} crossS={crossS:.2f} "
            f"vs={vertical_speed:.2f} hdg={heading:.1f} axis={axis_heading:.1f} "
            f"roll={roll} pitch={pitch} yaw=1500 thr={throttle}"
        )

        # ---------------------------------------------------------
        # Завершення
        # ---------------------------------------------------------
        if alt <= 0.7:
            self.set_rc(
                roll=1500,
                pitch=1500,
                throttle=1000,
                yaw=1500
            )
            print(f"LAND complete {'OFF target: toB=' + format(dist_to_B, '.1f') if dist_to_B > 2.0 else ''}".rstrip())
            self.state = State.DONE
            return 
            
    def step(self):
        s = self.read_state()

        if self.state == State.TAKEOFF:
            self.handle_takeoff(s)
        elif self.state == State.HOLD:
            self.handle_hold(s)
        elif self.state == State.CAPTURE_LINE:
            self.handle_capture_line(s)
        elif self.state == State.CRUISE:
            self.handle_cruise_along_line(s)
        elif self.state == State.APPROACH:
            self.handle_approach(s)
        elif self.state == State.LAND:
            self.handle_land(s)
        #elif self.state == State.FINAL_BRAKE:
        #    self.handle_final_brake(s)
        #elif self.state == State.PRE_FINAL_ALIGN:
        #    self.handle_pre_final_align(s)
        elif self.state == State.DONE:
            return False
        return True

    def run(self):
        is_worked = True
        
        try:
            self.initialize()

            while is_worked:
                is_worked = self.step()
                time.sleep(self.DT)

        finally:
            self.clear_rc()
            print("RC overrides очищено")


def main():
    print("Підключаємось...")
    
    #sitl = start_default(lat=50.450739, lon=30.461242)
    #vehicle = connect(sitl.connection_string(), wait_ready=True)
    #--home=50.450739,30.461242,180,0
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, heartbeat_timeout=30)
    #print("Підключаємось...")

    #sitl = start_default(lat=50.450739, lon=30.461242)
    #vehicle = connect(sitl.connection_string(), wait_ready=True)

    print("Чекаємо готовності автопілота...")
    while not vehicle.is_armable:
        time.sleep(1)

    time.sleep(3)

    for name in [
        "SIM_SPEEDUP",
        "SIM_WIND_SPD",
        "SIM_WIND_DIR",
        "SIM_WIND_TURB",
        #"SIM_WIND_TURB_FREQ",
    ]:
        try:
            print("BEFORE", name, vehicle.parameters[name])
        except Exception as e:
            print("BEFORE", name, "ERR", e)

    set_param_checked(vehicle, "SIM_SPEEDUP", 2)
    set_param_checked(vehicle, "SIM_WIND_SPD", 3)
    set_param_checked(vehicle, "SIM_WIND_DIR", 30)
    set_param_checked(vehicle, "SIM_WIND_TURB", 2)
    #set_param_checked(vehicle, "SIM_WIND_TURB_FREQ", 0.2)

    for name in [
        "SIM_SPEEDUP",
        "SIM_WIND_SPD",
        "SIM_WIND_DIR",
        "SIM_WIND_TURB",
        #"SIM_WIND_TURB_FREQ",
    ]:
        try:
            print("AFTER", name, vehicle.parameters[name])
        except Exception as e:
            print("AFTER", name, "ERR", e)
    fsm = DroneFSM(vehicle)

    try:
        fsm.run()
    finally:
        vehicle.close()
        print("З'єднання закрито")


if __name__ == "__main__":
    main()
