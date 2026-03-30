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

# Канали можуть відрізнятись у конкретній конфігурації, але зазвичай:
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
    
def clamp(value, lo, hi):
    return max(lo, min(hi, int(value)))


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
        self.vehicle = vehicle
        self.state = State.TAKEOFF

        self.target_lat = B_LAT
        self.target_lon = B_LON
        self.target_alt = TARGET_ALT

        self.hold_start = None
        self.start_lat = None
        self.start_lon = None
        
        self.cruise_start_lat = None
        self.cruise_start_lon = None
        
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
        
        self.prev_capture_abs_cross = None
        self.capture_no_progress_ticks = 0
        
        self.approach_phase = None
        self.approach_target_alt = None
        
        self.land_offset_lat = None
        self.land_offset_lon = None
        
        self.approach_min_target_alt = None
        self.land_heading_ref = None        
        self.prev_nav_lat = None
        self.prev_nav_lon = None
        
        self.land_axis_heading = None
                
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
            overrides[CH_ROLL] = clamp(roll, RC_MIN, RC_MAX)
        if pitch is not None:
            overrides[CH_PITCH] = clamp(pitch, RC_MIN, RC_MAX)
        if throttle is not None:
            overrides[CH_THROTTLE] = clamp(throttle, RC_MIN, RC_MAX)
        if yaw is not None:
            overrides[CH_YAW] = clamp(yaw, RC_MIN, RC_MAX)

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
        # Если wind_est_dir = куда сносит, то ось линии такая же.
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
        # >0 с одной стороны линии, <0 с другой
        cross = (apx * aby - apy * abx) / ab_len

        self.capture_line_bearing = normalize_angle_deg(line_bearing)
        self.capture_cross_sign = 1.0 if cross >= 0 else -1.0

        # Захоплюємо лінію поперечным движением:
        # если мы справа от линии, идём влево на 90°
        # если слева — вправо на 90°
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
            vertical_speed = (s['alt'] - self.prev_alt) / DT if self.prev_alt is not None else 0.0
            self.prev_alt = s['alt']

            base_takeoff_throttle = 1485
            throttle = base_takeoff_throttle - 3.0 * vertical_speed
            throttle = clamp(throttle, 1450, 1500)    
            
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

        vertical_speed = (s['alt'] - self.prev_alt) / DT if self.prev_alt is not None else 0.0
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
        throttle = clamp(throttle, 1300, 1525)

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
        # Для тесту тримаємо 15 секунд и выходим
        """
        if self.hold_start is not None and time.time() - self.hold_start > 15 and math.fabs(TARGET_ALT-self.alt_first)+math.fabs(TARGET_ALT-self.alt_last) < 1.0:
            print("Тест утримання завершено")
            self.state = State.CRUISE
            self.cruise_start_lat = self.vehicle.location.global_relative_frame.lat
            self.cruise_start_lon = self.vehicle.location.global_relative_frame.lon
        """
        # Умова "успокоились около цели"
        stable_alt = abs(error) < 1.0
        stable_vs = abs(vertical_speed) < 0.3

        if stable_alt and stable_vs:
            self.stable_hold_ticks += 1
        else:
            self.stable_hold_ticks = 0

        #if self.hold_start is not None and time.time() - self.hold_start > 10 and stable_alt and stable_vs:
        if self.hold_start is not None and time.time() - self.hold_start > 4 and self.stable_hold_ticks >= 4:
            print("Тест утримання завершено")
            self.prepare_capture_line()
            self.state = State.CAPTURE_LINE
            print("Перехід HOLD -> CAPTURE_LINE")
            self.cruise_start_lat = self.vehicle.location.global_relative_frame.lat
            self.cruise_start_lon = self.vehicle.location.global_relative_frame.lon
            self.prev_alt = None
            self.prev_capture_abs_cross = None
            self.capture_no_progress_ticks = 0

    def handle_capture_line(self, s):
        import math

        def to_local_m(lat0, lon0, lat1, lon1):
            dx = (lon1 - lon0) * 111320.0 * math.cos(math.radians(lat0))
            dy = (lat1 - lat0) * 111320.0
            return dx, dy

        def from_local_m(lat0, lon0, dx, dy):
            lat = lat0 + dy / 111320.0
            lon = lon0 + dx / (111320.0 * math.cos(math.radians(lat0)))
            return lat, lon

        vx, vy, vz = self.vehicle.velocity

        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        if self.wind_est_dir is None:
            print("[CAPTURE_LINE] no wind estimate")
            return

        # Лінія через B вздовж вітру
        line_bearing = self.wind_est_dir % 360.0
        ref2_lat, ref2_lon = self.offset_gps(B_LAT, B_LON, line_bearing, 100.0)

        # Локальна геометрія
        abx, aby = to_local_m(B_LAT, B_LON, ref2_lat, ref2_lon)
        apx, apy = to_local_m(B_LAT, B_LON, lat, lon)

        ab_len2 = abx * abx + aby * aby
        ab_len = math.sqrt(ab_len2)

        if ab_len < 1e-6:
            print("[CAPTURE_LINE] invalid geometry")
            return

        # signed cross-track для лога
        cross_track = (apx * aby - apy * abx) / ab_len
        abs_cross = abs(cross_track)

        # Найближча точка на лінії
        proj = (apx * abx + apy * aby) / ab_len2
        cx = proj * abx
        cy = proj * aby

        closest_lat, closest_lon = from_local_m(B_LAT, B_LON, cx, cy)

        # Курс у найближчу точку лінії
        target_heading = bearing_deg(lat, lon, closest_lat, closest_lon)

        # Форсажний увод трохи "глибше" в бік лінії, поки далеко
        if abs_cross > 300.0:
            heading_boost = 14.0
        elif abs_cross > 220.0:
            heading_boost = 10.0
        elif abs_cross > 140.0:
            heading_boost = 6.0
        else:
            heading_boost = 0.0

        if cross_track < 0:
            target_heading = normalize_angle_deg(target_heading - heading_boost)
        else:
            target_heading = normalize_angle_deg(target_heading + heading_boost)

        heading_error = normalize_angle_deg(target_heading - heading)
        ground_speed = math.hypot(vx, vy)

        # Контроль прогресу по cross-track
        if self.prev_capture_abs_cross is not None:
            if abs_cross < self.prev_capture_abs_cross - 0.5:
                self.capture_no_progress_ticks = 0
            else:
                self.capture_no_progress_ticks += 1
        else:
            self.capture_no_progress_ticks = 0

        self.prev_capture_abs_cross = abs_cross

        # Висота
        KP_ALT = 2.5
        KD_ALT = 8.0
        vertical_speed = (alt - self.prev_alt) / DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        throttle = self.hover_throttle + KP_ALT * (TARGET_ALT - alt) - KD_ALT * vertical_speed
        throttle = clamp(throttle, 1000, 1525)

        # ---------------------------------------------------------
        # ФОРСАЖНИЙ PITCH:
        # далеко від лінії -> активно прём до линии
        # ---------------------------------------------------------
        if abs_cross > 300.0:
            base_pitch = 1415
        elif abs_cross > 220.0:
            base_pitch = 1418
        elif abs_cross > 140.0:
            base_pitch = 1424
        elif abs_cross > 80.0:
            base_pitch = 1436
        else:
            base_pitch = 1450

        # Якщо довго немає нормального темпу, ще підсилюємо
        if self.capture_no_progress_ticks > 10:
            base_pitch -= 4
        if self.capture_no_progress_ticks > 20:
            base_pitch -= 4

        # Не даємо піти в зовсім божевільний форсаж
        base_pitch = max(1410, base_pitch)

        # Якщо швидкість по землі зовсім просіла - ще трохи додати
        if ground_speed < 2.8:
            base_pitch = max(1410, base_pitch - 4)

        roll = 1500
        pitch = base_pitch
        yaw = 1500
        mode = "CAPTURE"

        # Поки не довернули — активніше крутимо
        if abs(heading_error) > 20.0:
            roll = clamp(1500 + 3.8 * heading_error, 1455, 1545)
            yaw  = clamp(1500 + 10.0 * heading_error, 1430, 1570)
        else:
            roll = clamp(1500 + 4.5 * heading_error, 1455, 1545)
            yaw  = clamp(1500 + 8.8 * heading_error, 1435, 1565)

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        print(
            f"[CAPTURE_LINE/{mode}] "
            f"xtrack={cross_track:.1f} absx={abs_cross:.1f} "
            f"alt={alt:.1f} gs={ground_speed:.2f} "
            f"hdg={heading:.1f} thdg={target_heading:.1f} "
            f"herr={heading_error:.1f} "
            f"roll={roll} pitch={pitch} yaw={yaw} thr={throttle} "
            f"nprog={self.capture_no_progress_ticks}"
        )

        # Захопили лінію — тоді в CRUISE
        CAPTURE_XTRACK_OK = 7.0
        CAPTURE_CONFIRM_TICKS = 3

        if abs_cross <= CAPTURE_XTRACK_OK:
            self.capture_ok_ticks += 1
        else:
            self.capture_ok_ticks = 0

        if self.capture_ok_ticks >= CAPTURE_CONFIRM_TICKS:
            print("[CAPTURE_LINE] line captured -> CRUISE")
            self.capture_ok_ticks = 0
            self.state = State.CRUISE
            return
    
        if self.capture_no_progress_ticks > 40 and abs_cross > 80.0:
            print("[CAPTURE_LINE] slow progress, keep forcing capture")
            self.capture_no_progress_ticks = 20            
            
    def handle_cruise(self, s):
        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        vx, vy, vz = self.vehicle.velocity

        dist_to_target = distance_m(lat, lon, B_LAT, B_LON)
        bearing_to_target = bearing_deg(lat, lon, B_LAT, B_LON)
        ground_speed = math.hypot(vx, vy)

        # Базовий курс уздовж вітру
        wind_heading = self.wind_est_dir % 360.0

        to_b_err = normalize_angle_deg(bearing_to_target - wind_heading)

        if dist_to_target > 180.0:
            mix_gain = 0.45
            mix_limit = 22.0
        elif dist_to_target > 120.0:
            mix_gain = 0.35
            mix_limit = 18.0
        elif dist_to_target > 80.0:
            mix_gain = 0.28
            mix_limit = 14.0
        else:
            mix_gain = 0.20
            mix_limit = 10.0

        heading_correction = clamp(mix_gain * to_b_err, -mix_limit, mix_limit)
        target_heading = normalize_angle_deg(wind_heading + heading_correction)

        heading_error = normalize_angle_deg(target_heading - heading)

        br = math.radians(bearing_to_target)
        cls = vx * math.cos(br) + vy * math.sin(br)
        latv = -vx * math.sin(br) + vy * math.cos(br)

        # ---------------------------------------------------------
        # Цільова висота в CRUISE: ще трохи нижче
        # ---------------------------------------------------------
        if dist_to_target > 220.0:
            cruise_target_alt = 165.0
        elif dist_to_target > 170.0:
            cruise_target_alt = 128.0
        elif dist_to_target > 130.0:
            cruise_target_alt = 90.0
        elif dist_to_target > 95.0:
            cruise_target_alt = 52.0
        elif dist_to_target > 70.0:
            cruise_target_alt = 28.0
        else:
            cruise_target_alt = 14.0

        # ---------------------------------------------------------
        # Вертикаль
        # ---------------------------------------------------------
        KP_ALT = 3.2
        KD_ALT = 8.8
        vertical_speed = (alt - self.prev_alt) / DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        throttle = self.hover_throttle + KP_ALT * (cruise_target_alt - alt) - KD_ALT * vertical_speed
        throttle = clamp(throttle, 1000, 1525)

        # ---------------------------------------------------------
        # Roll/Yaw
        # ---------------------------------------------------------
        roll = clamp(1500 + 3.8 * heading_error, 1460, 1540)
        yaw  = clamp(1500 + 8.5 * heading_error, 1438, 1562)

        # ---------------------------------------------------------
        # Pitch
        # ---------------------------------------------------------
        pitch = 1450

        if cls < 2.6:
            pitch = 1438
        elif cls < 2.8:
            pitch = 1442
        elif cls < 3.0:
            pitch = 1446

        if abs(latv) > 1.4:
            pitch -= 4
        elif abs(latv) > 0.9:
            pitch -= 2

        pitch = clamp(pitch, 1434, 1500)

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        print(
            f"[WIND_CRUISE] "
            f"toB={dist_to_target:.1f} alt={alt:.1f}/{cruise_target_alt:.1f} "
            f"gs={ground_speed:.2f} cls={cls:.2f} latv={latv:.2f} "
            f"hdg={heading:.1f} thdg={target_heading:.1f} herr={heading_error:.1f}"
        )

        # У APPROACH заходимо нижче
        if dist_to_target <= 90.0 and alt <= 68.0:
            print("WIND_CRUISE -> APPROACH")
            self.state = State.APPROACH
            return
        
    def handle_approach(self, s):
        import math

        vx, vy, vz = self.vehicle.velocity

        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        dist_to_target = distance_m(lat, lon, B_LAT, B_LON)
        bearing_to_target = bearing_deg(lat, lon, B_LAT, B_LON)
        ground_speed = math.hypot(vx, vy)

        wind_heading = self.wind_est_dir % 360.0

        if self.prev_nav_lat is not None and self.prev_nav_lon is not None:
            dx = (lon - self.prev_nav_lon) * 111320.0 * math.cos(math.radians(lat))
            dy = (lat - self.prev_nav_lat) * 111320.0
            if abs(dx) + abs(dy) > 0.01:
                track_heading = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
            else:
                track_heading = heading
        else:
            track_heading = heading

        self.prev_nav_lat = lat
        self.prev_nav_lon = lon

        if dist_to_target > 70.0:
            align = 0.15
        elif dist_to_target > 50.0:
            align = 0.30
        elif dist_to_target > 35.0:
            align = 0.40
        elif dist_to_target > 22.0:
            align = 0.30
        elif dist_to_target > 14.0:
            align = 0.15
        else:
            align = 0.0

        course_delta = normalize_angle_deg(wind_heading - bearing_to_target)
        target_heading = normalize_angle_deg(bearing_to_target + align * course_delta)

        heading_error = normalize_angle_deg(target_heading - heading)
        move_error = normalize_angle_deg(bearing_to_target - track_heading)

        herrW = normalize_angle_deg(wind_heading - heading)
        herrT = normalize_angle_deg(bearing_to_target - heading)

        br = math.radians(bearing_to_target)
        cls = vx * math.cos(br) + vy * math.sin(br)
        latv = -vx * math.sin(br) + vy * math.cos(br)

        if dist_to_target > 55.0:
            raw_target_alt = 24.0
        elif dist_to_target > 35.0:
            raw_target_alt = 15.0
        elif dist_to_target > 20.0:
            raw_target_alt = 8.0
        elif dist_to_target > 12.0:
            raw_target_alt = 4.5
        else:
            raw_target_alt = 2.5

        if self.approach_min_target_alt is None:
            self.approach_min_target_alt = raw_target_alt
        else:
            self.approach_min_target_alt = min(self.approach_min_target_alt, raw_target_alt)

        approach_target_alt = self.approach_min_target_alt

        KP_ALT = 3.4
        KD_ALT = 9.0
        vertical_speed = (alt - self.prev_alt) / DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        throttle = self.hover_throttle + KP_ALT * (approach_target_alt - alt) - KD_ALT * vertical_speed
        throttle = clamp(throttle, 1000, 1500)

        steer_error = clamp(1.00 * heading_error, -18.0, 18.0)

        roll = clamp(1500 + 3.2 * steer_error - 2.8 * latv, 1485, 1535)
        yaw  = clamp(1500 + 4.5 * steer_error, 1465, 1535)

        if dist_to_target < 12.0:
            if cls < 1.4:
                pitch = 1508
            elif cls < 1.9:
                pitch = 1506
            else:
                pitch = 1504
        elif dist_to_target < 16.0:
            if cls < 1.8:
                pitch = 1507
            elif cls < 2.3:
                pitch = 1505
            else:
                pitch = 1503
        elif dist_to_target < 20.0:
            if cls < 2.2:
                pitch = 1502
            elif cls < 2.6:
                pitch = 1504
            else:
                pitch = 1506
        else:
            if cls < 2.5:
                pitch = 1495
            elif cls < 2.8:
                pitch = 1500
            else:
                pitch = 1505

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        print(
            f"[APPROACH/PATCH4] "
            f"toB={dist_to_target:.1f} alt={alt:.1f}/{approach_target_alt:.1f} "
            f"gs={ground_speed:.2f} cls={cls:.2f} latv={latv:.2f} "
            f"hdg={heading:.1f} trk={track_heading:.1f} "
            f"herr={heading_error:.1f} merr={move_error:.1f} "
            f"whdg={wind_heading:.1f} herrW={herrW:.1f} herrT={herrT:.1f}"
        )

        if not hasattr(self, "approach_overshoot_ticks"):
            self.approach_overshoot_ticks = 0

        if cls < -0.05:
            self.approach_overshoot_ticks += 1
        else:
            self.approach_overshoot_ticks = 0

        if dist_to_target <= 8.0 and alt <= 8.5:
            print("APPROACH -> LAND")
            self.land_axis_heading = heading
            self.approach_min_target_alt = None
            self.approach_overshoot_ticks = 0
            self.state = State.LAND
            return

        if self.approach_overshoot_ticks >= 2 and dist_to_target <= 16.0:
            print("APPROACH overshoot -> LAND")
            self.land_axis_heading = heading
            self.approach_min_target_alt = None
            self.approach_overshoot_ticks = 0
            self.state = State.LAND
            return
            
    def handle_land(self, s):
        import math

        vx, vy, vz = self.vehicle.velocity

        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        alt = self.vehicle.location.global_relative_frame.alt or 0.0
        heading = self.vehicle.heading or 0.0

        ground_speed = math.hypot(vx, vy)

        ex = (B_LON - lon) * 111320.0 * math.cos(math.radians(lat))
        ey = (B_LAT - lat) * 111320.0

        dist_to_target = math.hypot(ex, ey)
        bearing_to_target = bearing_deg(lat, lon, B_LAT, B_LON)

        if not hasattr(self, "land_axis_heading") or self.land_axis_heading is None:
            self.land_axis_heading = heading

        axis = math.radians(self.land_axis_heading)

        along_err = ex * math.sin(axis) + ey * math.cos(axis)
        cross_err = ex * math.cos(axis) - ey * math.sin(axis)

        along_speed = vx * math.sin(axis) + vy * math.cos(axis)
        cross_speed = vx * math.cos(axis) - vy * math.sin(axis)

        br = math.radians(bearing_to_target)
        cls = vx * math.cos(br) + vy * math.sin(br)

        vertical_speed = (alt - self.prev_alt) / DT if self.prev_alt is not None else 0.0
        self.prev_alt = alt

        # ---------------------------------------------------------
        # Тяга
        # ---------------------------------------------------------
        if alt > 6.0:
            throttle = 1456
        elif alt > 3.0:
            throttle = 1460
        elif alt > 1.5:
            throttle = 1462
        else:
            throttle = 1464

        if vertical_speed < -4.0:
            throttle += 6
        elif vertical_speed > -0.6:
            throttle -= 3

        throttle = clamp(throttle, 1000, 1485)

        # ---------------------------------------------------------
        # SMART FLARE
        # Не тільки по dist, а по фактичному стану
        # ---------------------------------------------------------
        smart_flare = (
            (dist_to_target < 8.0 and cls < 1.0) or
            (cls < 0.3) or
            (along_err < 1.0) or
            (alt < 4.0)
        )

        if not smart_flare:
            heading_error = normalize_angle_deg(self.land_axis_heading - heading)
            heading_error = clamp(heading_error, -12.0, 12.0)

            roll_cmd = 1500 + 0.8 * heading_error - 7.0 * cross_speed - 2.0 * cross_err
            yaw = clamp(1500 + 2.2 * heading_error, 1468, 1532)

            if along_speed > 1.5:
                pitch = 1490
            elif along_speed > 0.8:
                pitch = 1493
            else:
                pitch = 1496
        else:
            # -----------------------------------------------------
            # SMART AGGRESSIVE FLARE
            # -----------------------------------------------------
            roll_cmd = 1500 - 11.5 * cross_speed - 3.8 * cross_err
            yaw = 1500

            if alt > 4.0:
                if ground_speed > 2.0:
                    pitch = 1478
                elif ground_speed > 1.4:
                    pitch = 1482
                elif ground_speed > 0.8:
                    pitch = 1486
                else:
                    pitch = 1490
            elif alt > 2.0:
                if ground_speed > 1.8:
                    pitch = 1476
                elif ground_speed > 1.2:
                    pitch = 1480
                elif ground_speed > 0.7:
                    pitch = 1484
                else:
                    pitch = 1488
            else:
                if ground_speed > 1.5:
                    pitch = 1474
                elif ground_speed > 1.0:
                    pitch = 1478
                elif ground_speed > 0.6:
                    pitch = 1482
                else:
                    pitch = 1486

        roll = clamp(roll_cmd, 1440, 1560)

        self.set_rc(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )

        print(
            f"[LAND/SMART_FLARE] "
            f"toB={dist_to_target:.1f} alt={alt:.1f} "
            f"gs={ground_speed:.2f} cls={cls:.2f} "
            f"alongE={along_err:.2f} crossE={cross_err:.2f} "
            f"alongS={along_speed:.2f} crossS={cross_speed:.2f} "
            f"vs={vertical_speed:.2f} "
            f"hdg={heading:.1f} axis={self.land_axis_heading:.1f} "
            f"roll={roll} pitch={pitch} yaw={yaw} thr={throttle}"
        )

        if alt <= 0.8:
            print("LAND complete")
            self.set_rc(
                roll=1500,
                pitch=1500,
                throttle=1000,
                yaw=1500
            )
            self.land_axis_heading = None
            self.state = State.DONE            
            
    def step(self):
        s = self.read_state()

        if self.state == State.TAKEOFF:
            self.handle_takeoff(s)
        elif self.state == State.HOLD:
            self.handle_hold(s)
        elif self.state == State.CAPTURE_LINE:
            self.handle_capture_line(s)
        elif self.state == State.CRUISE:
            self.handle_cruise(s)
        elif self.state == State.APPROACH:
            self.handle_approach(s)
        elif self.state == State.LAND:
            self.handle_land(s)
        elif self.state == State.DONE:
            return False
        return True

    def run(self):
        is_worked = True
        
        try:
            self.initialize()

            while is_worked:
                is_worked = self.step()
                time.sleep(DT)

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
        #"SIM_SPEEDUP",
        "SIM_WIND_SPD",
        "SIM_WIND_DIR",
        "SIM_WIND_TURB",
        "SIM_WIND_TURB_FREQ",
    ]:
        try:
            print("BEFORE", name, vehicle.parameters[name])
        except Exception as e:
            print("BEFORE", name, "ERR", e)

    #set_param_checked(vehicle, "SIM_SPEEDUP", 2)
    set_param_checked(vehicle, "SIM_WIND_SPD", 3)
    set_param_checked(vehicle, "SIM_WIND_DIR", 30)
    set_param_checked(vehicle, "SIM_WIND_TURB", 2)
    set_param_checked(vehicle, "SIM_WIND_TURB_FREQ", 0.2)

    for name in [
        #"SIM_SPEEDUP",
        "SIM_WIND_SPD",
        "SIM_WIND_DIR",
        "SIM_WIND_TURB",
        "SIM_WIND_TURB_FREQ",
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