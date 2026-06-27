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
#   BS_ALT (3)        — целевая высота, м
#   BS_HANDOVER (0)   — 1 = перейти в GUIDED после init (иначе OBSERVE→LAND)
#   BS_EXCITE (80)    — амплитуда раскачки roll/pitch, PWM от центра
#   BS_OBSERVE (15)   — держать высоту после init перед посадкой, sim-сек (без handover)
#   BS_VINS_TO (90)   — таймаут ожидания сходимости VINS, sim-сек
set -e
source /opt/ros/humble/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

ALT="${BS_ALT:-3}"
EXCITE="${BS_EXCITE:-80}"
OBSERVE="${BS_OBSERVE:-15}"
VINS_TO="${BS_VINS_TO:-90}"

ARGS=(--alt "$ALT" --excite "$EXCITE" --observe "$OBSERVE" --vins-timeout "$VINS_TO")
[ "${BS_HANDOVER:-0}" = "1" ] && ARGS+=(--handover)

echo ">>> ALT_HOLD bootstrap: alt=${ALT}м excite=±${EXCITE}PWM handover=${BS_HANDOVER:-0} (sim-время)..."
python3 /lab/alt_hold_bootstrap.py "${ARGS[@]}"
echo ">>> bootstrap завершён."
