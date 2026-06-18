# ============================================================================
# geo.py — чистая геометрия для ray tracing NN1 (без ROS, тестируемо отдельно).
#
# Рамки:
#   ENU         — локальная метрическая, начало = точка взлёта (датум из БД).
#                 X=East, Y=North, Z=Up. Совпадает с рамкой VINS (по init).
#   body (FLU)  — корпус дрона: X вперёд, Y влево, Z вверх (REP-103).
#                 R_enu_body берём из ориентации /mavros/imu/data.
#   camlink     — звено камеры в Gazebo: X вперёд, Y влево, Z вверх,
#                 повёрнуто относительно body на cam_mount_rpy (из model.sdf).
#   optical(CV) — оптическая рамка камеры: X вправо, Y вниз, Z вперёд.
#                 В ней работает обратная проекция пикселя.
# ============================================================================
import numpy as np

R_EARTH = 6378137.0   # WGS84, м

# Оси optical(CV) выраженные в camlink (Gazebo): постоянная конвенция.
#   optical X (вправо) = camlink -Y
#   optical Y (вниз)   = camlink -Z
#   optical Z (вперёд) = camlink +X
R_CAMLINK_OPT = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """GPS -> локальные ENU-метры относительно датума (равнопромежуточная
    аппроксимация — точна на масштабах км; для больших площадей взять pymap3d)."""
    dlat = np.radians(lat - lat0)
    dlon = np.radians(lon - lon0)
    e = dlon * R_EARTH * np.cos(np.radians(lat0))
    n = dlat * R_EARTH
    u = alt - alt0
    return np.array([e, n, u])


def quat_to_rotmat(x, y, z, w):
    """ROS-кватернион (x,y,z,w) -> 3x3 (body -> world для ориентации IMU)."""
    nrm = x * x + y * y + z * z + w * w
    if nrm < 1e-12:
        return np.eye(3)
    s = 2.0 / nrm
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def rpy_to_rotmat(roll, pitch, yaw):
    """Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def backproject(u, v, fx, fy, cx, cy):
    """Пиксель -> единичный луч в optical(CV)-рамке (Z вперёд)."""
    d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    return d / np.linalg.norm(d)


def solve_camera_position(P, ray_world, cam_z):
    """Луч из камеры через пиксель: P = C + t*ray_world. Известна высота камеры
    cam_z (баро) -> решаем XY камеры. Возвращает C (ENU) или None.

    P         — известная точка ориентира в ENU (м).
    ray_world — направление луча в ENU (от камеры в сцену), единичный.
    cam_z     — высота камеры (ENU U) над датумом.
    """
    rz = ray_world[2]
    if abs(rz) < 1e-6:          # луч горизонтален — пересечения с плоскостью нет
        return None
    t = (P[2] - cam_z) / rz
    if t <= 0:                  # ориентир «позади» камеры — отбраковка
        return None
    C = P - t * ray_world
    return np.array([C[0], C[1], cam_z])   # Z жёстко = cam_z
