#!/usr/bin/env bash
#
# vins_offline_replay.sh — офлайн-прогон vins_estimator по записанному бэгу.
#
# Гоняет ТОЛЬКО estimator (без Gazebo/камеры/трекера) по /feature + IMU из бэга
# в ИЗОЛИРОВАННОМ ROS-домене (ROS_DOMAIN_ID=42) — живой стек не видит реплей и
# наоборот. Один прогон ≈ длительность бэга / RATE. Это главный инструмент
# итераций по параметрам VINS: правка конфига → реплей → метрики, без полётов.
# Дисциплину прогонов не нарушает: стек не трогаем, только читаем бэг.
#
# Именно этим стендом 2026-08-19 найден корень развала VINS: в конфигах не было
# max_solver_time/max_num_iterations → ceres 0 итераций → солвер-пустышка.
#
# Требования к бэгу: топики /feature (sensor_msgs/PointCloud от feature_tracker)
# и IMU из imu_topic конфига (/gz_imu/data_flu). Опц. /model/iris_cam/odometry
# (истина Gazebo) — реплеится и дампится для сравнения масштаба/дрейфа.
#
# Запускать С ХОСТА:
#   bash src/lab/vins_offline_replay.sh <имя> [BAG] [SED_OVERRIDES...]
#     имя   — метка прогона (логи/дампы в /tmp/offline/ контейнера nav)
#     BAG   — путь бэга В КОНТЕЙНЕРЕ (default /root/sim_ws/output/VINSEXT_bag)
#     SED_OVERRIDES — пары key value: перекрыть скаляр конфига, напр.:
#       bash src/lab/vins_offline_replay.sh noise020 '' acc_n 0.2 gyr_n 0.02
#
# База конфига — /tmp/sim_960x540.yaml в контейнере (масштабированный sim.yaml
# последнего запуска стека). RATE=2 (env) — скорость реплея.
#
# Результаты в контейнере nav:
#   /tmp/offline/est_<имя>.log   — лог estimator (init/failure/td/ric)
#   /tmp/offline/dump_<имя>.csv  — vins,t,x,y,z,vx,vy,vz + truth,... построчно
set -eo pipefail
NAME=${1:?имя прогона}
BAG=${2:-/root/sim_ws/output/VINSEXT_bag}
[ -z "$BAG" ] && BAG=/root/sim_ws/output/VINSEXT_bag
shift 2 2>/dev/null || shift $#
RATE=${RATE:-2}

SEDS=""
while [ $# -ge 2 ]; do
    SEDS="$SEDS -e s|^$1:.*|$1: $2|"
    shift 2
done

docker exec -i -e NAME="$NAME" -e BAG="$BAG" -e RATE="$RATE" -e SEDS="$SEDS" \
    p1317_nav bash -s <<'INNER'
set -e
source /opt/ros/humble/setup.bash
source /opt/overlay/install/setup.bash
source /root/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=42
mkdir -p /tmp/offline
CFG=/tmp/offline/cfg_$NAME.yaml
# output_path уводим в /tmp/offline, чтобы не клоббер живого CSV
sed $SEDS -e "s|output_path:.*|output_path: \"/tmp/offline/\"|" \
    /tmp/sim_960x540.yaml > $CFG

ros2 run vins_estimator vins_estimator --ros-args \
    -p config_file:=$CFG \
    -r /feature_tracker/feature:=/feature -r /feature_tracker/restart:=/restart \
    > /tmp/offline/est_$NAME.log 2>&1 &
EST=$!

python3 - > /tmp/offline/dump_$NAME.csv 2>/tmp/offline/dump_$NAME.err <<'PYEOF' &
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
rclpy.init()
n = Node('dumper')
def cb(tag):
    def f(m):
        p = m.pose.pose.position; v = m.twist.twist.linear
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        print("%s,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"
              % (tag, t, p.x, p.y, p.z, v.x, v.y, v.z), flush=True)
    return f
n.create_subscription(Odometry, '/odometry', cb('vins'), 100)
n.create_subscription(Odometry, '/model/iris_cam/odometry', cb('truth'), 100)
rclpy.spin(n)
PYEOF
DUMP=$!

sleep 4
ros2 bag play "$BAG" --rate $RATE \
    --topics /feature /gz_imu/data_flu /model/iris_cam/odometry >/dev/null 2>&1
sleep 3
kill $EST $DUMP 2>/dev/null || true
echo "=== $NAME: init=$(grep -c 'Initialization finish' /tmp/offline/est_$NAME.log || true)" \
     "fail=$(grep -c 'failure detection' /tmp/offline/est_$NAME.log || true)" \
     "причины: $(grep -o 'big [a-z ]*' /tmp/offline/est_$NAME.log | sort | uniq -c | tr '\n' ' ')"
INNER
