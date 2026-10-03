#!/usr/bin/env bash
# cmd/cam_tilt/cam_tilt.sh — A/B НАКЛОНА КАМЕРЫ ВНИЗ для канала вида сверху (2026-10-03). Зачем — README.txt.
#
#   bash cmd/cam_tilt/cam_tilt.sh 15      # камера наклонена вниз на 15°, остальное — штат cmd/ipm_lever wl
#   bash cmd/cam_tilt/cam_tilt.sh 25
#   bash cmd/cam_tilt/cam_tilt.sh 0       # контроль (= бортовой конфиг)
#
# Бортовой config.yaml НЕ трогается: копия с повёрнутой extrinsicRotation (ось Z камеры опущена
# на θ вокруг поперечной оси корпуса, X камеры = −Y корпуса) кладётся в docker/sim/output/cam_tilt/
# и отдаётся прогону через CAM_CFG (compose берёт env оболочки поверх .env; freefly_lv делает
# fresh-start). Из неё же — поза camera_link в Gazebo, VINS сима, наклон демпфера.
# Сценарий и профили — cmd/ipm_lever/ipm_lever.sh wl, ветер world/baseline (5 м/с).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
DEG="${1:?наклон вниз, градусы: 0|15|25|…}"
SRC=distro/home/andriy/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/config.yaml
DST=docker/sim/output/cam_tilt/config_tilt${DEG}.yaml
mkdir -p "$(dirname "$DST")"
python3 - "$SRC" "$DST" "$DEG" <<'PY'
import math, sys
sys.path.insert(0, 'src/control')
from control_pkg.perception import camera_mount as cm
src, dst, deg = sys.argv[1], sys.argv[2], float(sys.argv[3])
s, c = math.sin(math.radians(deg)), math.cos(math.radians(deg))
# imu^R_cam, СТОЛБЦЫ — оси камеры в теле FLU: X_cam=(0,−1,0), Y_cam=(−s,0,−c), Z_cam=(c,0,−s)
R = [[0, -s, c], [-1, 0, 0], [0, -c, -s]]
text = cm.replace_matrix(open(src).read(), 'extrinsicRotation', R)
# пометка — ПОСЛЕ первой строки: cv2.FileStorage (ray_tracer, geo.load_camera_mount) требует %YAML:1.0 первой
head, _, rest = text.partition('\n')
open(dst, 'w').write(f'{head}\n# СГЕНЕРИРОВАНО cmd/cam_tilt/cam_tilt.sh из {src}: камера наклонена вниз на {deg:g}°\n{rest}')
m = cm.CameraMount.load(dst)
print(f'cam_tilt: {m.summary()}')
assert abs(math.degrees(m.tilt) - deg) < 1e-6, m.tilt
PY
export CAM_CFG="$DST"
export WORLD_PROF=world/baseline
export NAME="${NAME:-tilt${DEG}_wl_baseline_$(date +%Y%m%d_%H%M%S)}"
exec bash cmd/ipm_lever/ipm_lever.sh wl
