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
[ -n "${BS_THROTTLE_CLIMB:-}" ] && ARGS+=(--throttle-climb "$BS_THROTTLE_CLIMB")
[ -n "${BS_MODE_BUDGET:-}" ]  && ARGS+=(--mode-budget "$BS_MODE_BUDGET")
[ -n "${BS_ARM_BUDGET:-}" ]   && ARGS+=(--arm-budget "$BS_ARM_BUDGET")
[ -n "${BS_CLIMB_BUDGET:-}" ] && ARGS+=(--climb-budget "$BS_CLIMB_BUDGET")
[ -n "${BS_LAND_BUDGET:-}" ]  && ARGS+=(--land-budget "$BS_LAND_BUDGET")

echo ">>> liftland (ALT_HOLD): alt=${ALT}м hold=${HOLD}s — БЕЗ раскачки, держим уровень..."
python3 /lab/alt_hold_bootstrap.py "${ARGS[@]}"
echo ">>> liftland завершён."
