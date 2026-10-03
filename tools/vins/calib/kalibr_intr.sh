#!/bin/bash
# kalibr_intr.sh — интринсики бортовой камеры из bag'ов rec_intr.sh (шаги 2–3 README.md).
#
#   ./kalibr_intr.sh <bag_dir> [<bag_dir> ...]     ros2-каталоги (docker/sim/output/board/calib/…)
#
# Несколько bag'ов сливаются в один ROS1-bag (rosbags-convert, venv ~/.venvs/rosbags) →
# kalibr_calibrate_cameras (образ kalibr:1f60227, модель pinhole-radtan = PINHOLE VINS).
# Результат — в каталог первого bag'а: <имя>.bag, *-camchain.yaml, *-results-cam.txt,
# *-report-cam.pdf, kalibr.log. Ручки: FREQ (Гц выборки кадров, умолч. 4), TARGET (yaml мишени).
#
# Venv: python3 -m venv ~/.venvs/rosbags && ~/.venvs/rosbags/bin/pip install rosbags

set -euo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
TARGET="${TARGET:-$HERE/aprilgrid_pvc1000.yaml}"
FREQ="${FREQ:-4}"
CONV=~/.venvs/rosbags/bin/rosbags-convert
[ $# -ge 1 ] || { sed -n '2,12p' "$0"; exit 1; }
[ -x "$CONV" ] || { echo "!! нет $CONV — см. шапку"; exit 1; }

SRC=()
for b in "$@"; do SRC+=("$(readlink -f "$b")"); done
OUT="${SRC[0]}"
NAME="$(basename "$OUT")"; [ $# -gt 1 ] && NAME="${NAME}_x$#"
BAG="$OUT/$NAME.bag"

rm -f "$BAG"
"$CONV" --src "${SRC[@]}" --dst "$BAG" --include-topic /image_mono --dst-typestore ros1_noetic
echo "== $BAG ($(du -h "$BAG" | cut -f1))"

cp "$TARGET" "$OUT/target.yaml"
docker run --rm -v "$OUT":/data --entrypoint bash kalibr:1f60227 -c \
  "source /catkin_ws/devel/setup.bash && cd /data && \
   rosrun kalibr kalibr_calibrate_cameras --bag $NAME.bag --topics /image_mono \
     --models pinhole-radtan --target target.yaml --bag-freq $FREQ --dont-show-report; \
   chown -R $(id -u):$(id -g) /data" \
  2>&1 | grep -v -E 'Unable to init server|Gdk-CRITICAL|^$' | tee "$OUT/kalibr.log"
ls "$OUT" | grep -E "$NAME-(camchain|results|report)" || echo "!! Kalibr не выдал результат — kalibr.log"
