#!/usr/bin/env bash
# Команда `liftland` — ALT_HOLD: взлёт → держать УРОВЕНЬ (без всякой раскачки) →
# посадка. «Просто взлетаем и садимся, никуда не летим».
#
# Зачем отдельно от `bootstrap`: изолирует ДРЕЙФ ALT_HOLD (остаточная скорость /
# наклон AHRS) от excite-раскачки. Если дрон при нулевом excite всё равно уезжает
# за край сцены — причина в AHRS-уровне/скорости, а не в управлении. Реюзает всю
# машинерию alt_hold_bootstrap.py (arm/climb/land/непрерывный RC override) в режиме
# --hold-only: фаза EXCITE заменяется на «держать центр стиков hold_sec sim-сек».
#
# Запуск внутри nav:  docker exec p1317_nav bash /lab/liftland.sh
# В секвенсоре:        bash src/lab/capture_scene.sh 960x540 liftland
#
# Параметры через env:
#   BS_ALT (3)         — целевая высота, м
#   BS_HOLD_SEC (30)   — сколько держать уровень перед посадкой, sim-сек
# Бюджеты/газ — те же, что у bootstrap (climb/arm/mode/land budgets, throttle-climb).
set -e
source /opt/ros/humble/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

ALT="${BS_ALT:-3}"
HOLD="${BS_HOLD_SEC:-30}"

ARGS=(--hold-only --alt "$ALT" --hold-sec "$HOLD")
# gz-position-hold (СИМ-костыль): держать точку по истинной позе Gazebo
[ "${BS_GZHOLD:-0}" = "1" ] && ARGS+=(--gz-hold)
[ -n "${BS_GZ_KP:-}" ]    && ARGS+=(--gz-kp "$BS_GZ_KP")
[ -n "${BS_GZ_KD:-}" ]    && ARGS+=(--gz-kd "$BS_GZ_KD")
[ -n "${BS_GZ_KI:-}" ]    && ARGS+=(--gz-ki "$BS_GZ_KI")
[ -n "${BS_GZ_IMAX:-}" ]  && ARGS+=(--gz-imax "$BS_GZ_IMAX")
[ -n "${BS_GZ_MAX:-}" ]   && ARGS+=(--gz-max "$BS_GZ_MAX")
[ -n "${BS_GZ_PSIGN:-}" ] && ARGS+=(--gz-psign "$BS_GZ_PSIGN")
[ -n "${BS_GZ_RSIGN:-}" ] && ARGS+=(--gz-rsign "$BS_GZ_RSIGN")
[ -n "${BS_GZ_TRAJ_R:-}" ] && ARGS+=(--gz-traj-r "$BS_GZ_TRAJ_R")
[ -n "${BS_GZ_TRAJ_T:-}" ] && ARGS+=(--gz-traj-t "$BS_GZ_TRAJ_T")
[ -n "${BS_GZ_YAW:-}" ]    && ARGS+=(--gz-yaw "$BS_GZ_YAW")
[ -n "${BS_GZ_YAW_PERIOD:-}" ] && ARGS+=(--gz-yaw-period "$BS_GZ_YAW_PERIOD")
# FLOW-DAMP (--flow-hold): боковой демпфер по камере вместо gz-истины
[ "${BS_FLOWHOLD:-0}" = "1" ] && ARGS+=(--flow-hold)
[ -n "${BS_FLOW_KP:-}" ]    && ARGS+=(--flow-kp "$BS_FLOW_KP")
[ -n "${BS_FLOW_KI:-}" ]    && ARGS+=(--flow-ki "$BS_FLOW_KI")
[ -n "${BS_FLOW_KD:-}" ]    && ARGS+=(--flow-kd "$BS_FLOW_KD")
[ -n "${BS_FLOW_IMAX:-}" ]  && ARGS+=(--flow-imax "$BS_FLOW_IMAX")
[ -n "${BS_FLOW_MAX:-}" ]   && ARGS+=(--flow-max "$BS_FLOW_MAX")
[ -n "${BS_FLOW_CONF_MIN:-}" ]  && ARGS+=(--flow-conf-min "$BS_FLOW_CONF_MIN")
[ -n "${BS_FLOW_CONF_FULL:-}" ] && ARGS+=(--flow-conf-full "$BS_FLOW_CONF_FULL")
[ -n "${BS_FLOW_RSIGN:-}" ] && ARGS+=(--flow-rsign "$BS_FLOW_RSIGN")
[ -n "${BS_FLOW_OSIGN:-}" ] && ARGS+=(--flow-osign "$BS_FLOW_OSIGN")
[ -n "${BS_FLOW_SMOOTH:-}" ] && ARGS+=(--flow-smooth "$BS_FLOW_SMOOTH")
# ROLL-EXCITE (--roll-excite): открытый контур для system-ID (калибровка flow_calib)
[ "${BS_ROLL_EXCITE:-0}" = "1" ] && ARGS+=(--roll-excite)
[ -n "${BS_RE_MODE:-}" ]   && ARGS+=(--roll-excite-mode "$BS_RE_MODE")
[ -n "${BS_RE_TAU:-}" ]    && ARGS+=(--roll-excite-tau "$BS_RE_TAU")
[ -n "${BS_RE_NREP:-}" ]   && ARGS+=(--roll-excite-nrep "$BS_RE_NREP")
[ -n "${BS_RE_AMP:-}" ]    && ARGS+=(--roll-excite-amp "$BS_RE_AMP")
[ -n "${BS_RE_F0:-}" ]     && ARGS+=(--roll-excite-f0 "$BS_RE_F0")
[ -n "${BS_RE_F1:-}" ]     && ARGS+=(--roll-excite-f1 "$BS_RE_F1")
[ -n "${BS_RE_CHIRP:-}" ]  && ARGS+=(--roll-excite-chirp "$BS_RE_CHIRP")
[ -n "${BS_RE_STEP:-}" ]   && ARGS+=(--roll-excite-step "$BS_RE_STEP")
[ -n "${BS_THROTTLE_CLIMB:-}" ] && ARGS+=(--throttle-climb "$BS_THROTTLE_CLIMB")
[ -n "${BS_MODE_BUDGET:-}" ]  && ARGS+=(--mode-budget "$BS_MODE_BUDGET")
[ -n "${BS_ARM_BUDGET:-}" ]   && ARGS+=(--arm-budget "$BS_ARM_BUDGET")
[ -n "${BS_CLIMB_BUDGET:-}" ] && ARGS+=(--climb-budget "$BS_CLIMB_BUDGET")
[ -n "${BS_LAND_BUDGET:-}" ]  && ARGS+=(--land-budget "$BS_LAND_BUDGET")

echo ">>> liftland (ALT_HOLD): alt=${ALT}м hold=${HOLD}s — БЕЗ раскачки, держим уровень..."
python3 /lab/alt_hold_bootstrap.py "${ARGS[@]}"
echo ">>> liftland завершён."
