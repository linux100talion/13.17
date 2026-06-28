#!/usr/bin/env bash
# Команда `bootstrap` — взлёт в ALT_HOLD и инициализация VINS в полёте (без GPS).
# Обёртка над alt_hold_bootstrap.py для секвенсора capture_scene и make.
#
# Запуск внутри nav-контейнера:  docker exec p1317_nav bash /lab/bootstrap.sh
# Через make: make bootstrap. В секвенсоре: capture_scene.sh ... bootstrap ...
#
# ВАЖНО: bootstrap САМ владеет всей лётной фазой (arm → climb → раскачка → ждёт
# VINS), т.к. ALT_HOLD требует непрерывного RC override (см. alt_hold_bootstrap.py).
# Поэтому ПЕРЕД ним НЕ нужен arm/takeoff. Чем заканчивается:
#   - по умолчанию (BS_HANDOVER!=1): сам садится (OBSERVE→LAND) — самодостаточно,
#     дрон на земле; в секвенсоре после bootstrap НЕ добавлять land;
#   - BS_HANDOVER=1: после сходимости VINS переходит в GUIDED и оставляет дрон в
#     воздухе (наблюдаем рывок) → дальше можно square/hover/land.
#
# Параметры через env (как SQ_* у square.sh):
#   BS_ALT (3)            — целевая высота, м
#   BS_HANDOVER (0)       — 1 = перейти в GUIDED после init (иначе OBSERVE→LAND)
#   BS_EXCITE (80)        — амплитуда раскачки forward/back, PWM от центра (масштаб радиуса)
#   BS_YAW (30)           — амплитуда медленного yaw в EXCITE, PWM от центра (0=без yaw)
#   BS_EXCITE_PERIOD (3)  — базовая τ профиля раскачки +τ/−2τ/+τ, sim-сек (цикл=4τ)
#   BS_OBSERVE (15)       — держать высоту после init перед посадкой, sim-сек (без handover)
#   BS_VINS_TO (90)       — таймаут ожидания сходимости VINS, sim-сек
# Бюджеты фаз автомата (sim-сек) — поднимать на низком RTF, как ARM_SIM_BUDGET у arm.sh
# (EKF/латч/арм/набор высоты не успевают в дефолт при llvmpipe/lockstep). Пусто = дефолт ноды:
#   BS_MODE_BUDGET (40)  — латч ALT_HOLD/GUIDED
#   BS_ARM_BUDGET (40)   — арминг
#   BS_CLIMB_BUDGET (60) — набор высоты до BS_ALT
#   BS_LAND_BUDGET (120) — посадка
set -e
source /opt/ros/humble/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

ALT="${BS_ALT:-3}"
EXCITE="${BS_EXCITE:-80}"
YAW="${BS_YAW:-30}"
EXCITE_PERIOD="${BS_EXCITE_PERIOD:-3}"
OBSERVE="${BS_OBSERVE:-15}"
VINS_TO="${BS_VINS_TO:-90}"

ARGS=(--alt "$ALT" --excite "$EXCITE" --yaw-rate "$YAW" --excite-period "$EXCITE_PERIOD"
      --observe "$OBSERVE" --vins-timeout "$VINS_TO")
[ "${BS_HANDOVER:-0}" = "1" ] && ARGS+=(--handover)
# Газ на подъём (PWM). Дефолт ноды 1650 ≈ висение для iris → набор маргинальный;
# поднять (напр. 1800) для уверенной скороподъёмности в ALT_HOLD. Пусто = дефолт ноды.
[ -n "${BS_THROTTLE_CLIMB:-}" ] && ARGS+=(--throttle-climb "$BS_THROTTLE_CLIMB")
# Бюджеты фаз — добавляем флаг только если env задан (иначе argparse-дефолт ноды):
[ -n "${BS_MODE_BUDGET:-}" ]  && ARGS+=(--mode-budget "$BS_MODE_BUDGET")
[ -n "${BS_ARM_BUDGET:-}" ]   && ARGS+=(--arm-budget "$BS_ARM_BUDGET")
[ -n "${BS_CLIMB_BUDGET:-}" ] && ARGS+=(--climb-budget "$BS_CLIMB_BUDGET")
[ -n "${BS_LAND_BUDGET:-}" ]  && ARGS+=(--land-budget "$BS_LAND_BUDGET")

echo ">>> ALT_HOLD bootstrap: alt=${ALT}м excite=±${EXCITE}PWM yaw=±${YAW}PWM handover=${BS_HANDOVER:-0} (sim-время)..."
python3 /lab/alt_hold_bootstrap.py "${ARGS[@]}"
echo ">>> bootstrap завершён."
