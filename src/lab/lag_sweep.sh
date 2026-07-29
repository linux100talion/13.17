#!/usr/bin/env bash
#
# lag_sweep.sh — СВИП ручек, отвечающих за СКОРОСТЬ РЕАКЦИИ продольного контура.
#
# Задача. Демпфер по тангажу «ватный»: борт едет назад 2.7 м/с, а команда — треугольная
# пила со средним −9 PWM (наклон −0.8°, ускорение −0.135 м/с² — в ту же сторону, куда
# едет). Замер задержек по D1c (back_check.py, взаимная корреляция, окно 9..18с):
#
#   v_fwd → flow_lon   (перцепт)  +0.58 с   ← медиана по 25 кадрам при 20 Гц
#   flow_lon → команда (контур)   +0.10 с   ← слю-лимит 100 PWM/с
#   команда → тангаж   (борт)     +0.20 с   ← τ=0.27 с, ФИЗИКА, не ручка
#   v_fwd → тангаж     (ВСЕГО)    +1.04 с
#
# Ручки, которые эту секунду сокращают, ровно три:
#   BS_SLEW          — потолок скорости команды, PWM/с (0 = выкл). В D1c команда
#                      упиралась в него 59% выборок: PID пересчитывает цель быстрее,
#                      чем ограничитель успевает к ней ехать → выход = пила со средним
#                      около нуля. Борт сам себе ФНЧ (τ=0.27), так что дребезг выше
#                      ~1 Гц он гасит и без ограничителя — тот дублировал фильтрацию,
#                      а платил задержкой.
#   BS_PITCH_SMOOTH  — медиана перцепта по N кадрам. Окно N/20 сек, ступень отдаётся
#                      за полокна. Самое большое звено (0.58 с из 1.04).
#   BS_PITCH_KD      — не сокращает задержку, а КОМПЕНСИРУЕТ: даёт опережение kd/kp сек.
#
# Чем платим за сглаживание. Шум сырого flow_lon σ=1.62 px; медиана по 25 даёт 0.32
# (замерено ровно столько). Снять сглаживание, не тронув kp, — вернуть шум в команду
# (kp·σ = 324 PWM = постоянный упор в потолок). Поэтому фаза 2 идёт ПО ДИАГОНАЛИ:
# сглаживание вниз и kp тем же множителем (∝1/√N), чтобы шум команды остался ~30 PWM
# СКО, как в D1c. Тогда сравниваются задержки, а не громкость.
#
# Фазы (одна ручка за прогон, победитель едет дальше):
#   1. слю-лимит:   300, затем 0 (выкл)          — то, что вчера пережали
#   2. сглаживание: 12/kp135/ki27, 5/kp88/ki18   — на победившем слю
# Фаза 2 заякорена на SLEW_BEST (по умолчанию 300). Если победит другой — правь
# переменную и перезапускай с START=3.
#
# Метрика — из bag'а каждого прогона (back_check.py дописывает строку в
# output/lag_sweep.csv): четыре задержки, % упора в слю-лимит, постоянная и переменная
# составляющие команды, средний наклон, средняя/макс скорость в висении. Задержки
# меряются ВНУТРИ прогона, поэтому разброс стенда (±40% по исходу) их не смазывает.
#
# Запуск (с хоста, отвязанно — 4 прогона по 7–15 мин):
#   cd /root/13.17 && setsid nohup bash src/lab/lag_sweep.sh \
#     > /root/run_lagsweep.log 2>&1 < /dev/null & disown
#
#   DRY_RUN=1 bash src/lab/lag_sweep.sh    — показать план, стек не трогать
#   START=3 bash src/lab/lag_sweep.sh      — начать с 3-й конфигурации
#   ONLY="F2_slew0 F4_sm5" bash …          — прогнать только эти (добор после сбоя)
#
set -uo pipefail        # без -e: падение одного прогона не должно рвать свип

START="${START:-1}"
DRY_RUN="${DRY_RUN:-0}"
SLEW_BEST="${SLEW_BEST:-300}"       # якорь фазы 2 (см. шапку)
ONLY="${ONLY:-}"                    # список имён через пробел = гнать только их (добор упавших)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN="$SCRIPT_DIR/calib_run.sh"
OUT="$REPO_ROOT/docker/sim/output"
CSV="$OUT/lag_sweep.csv"

# База — ровно D1c (единственное, что меняется, вынесено в CONFIGS).
export BS_STAB="${BS_STAB:-DpRollHold+DpPitchHold}"
export BS_MISSION="${BS_MISSION:-climb3,hover10,land}"
export BS_ROLL_KP="${BS_ROLL_KP:-16}"
export BS_ROLL_KI="${BS_ROLL_KI:-3}"
export BS_ROLL_IMAX="${BS_ROLL_IMAX:-150}"
export BS_ROLL_SMOOTH="${BS_ROLL_SMOOTH:-25}"
export BS_ROLL_OSIGN="${BS_ROLL_OSIGN:--1}"
export BS_PITCH_IMAX="${BS_PITCH_IMAX:-150}"
export BS_PITCH_OSIGN="${BS_PITCH_OSIGN:-1}"

# имя|slew|pitch_smooth|pitch_kp|pitch_ki|pitch_kd
CONFIGS=(
    "F1_slew300|300|25|200|40|0"
    "F2_slew0|0|25|200|40|0"
    "F3_sm12|$SLEW_BEST|12|135|27|0"
    "F4_sm5|$SLEW_BEST|5|88|18|0"
)

echo "############ СВИП ЗАДЕРЖКИ: ${#CONFIGS[@]} прогонов, с ${START}-го ############"
date
printf '%s\n' "эталон — D1c (slew 100, smooth 25, kp 200, ki 40): задержка 1.04 с, команда −9.4±29.6 PWM"

i=0
for cfg in "${CONFIGS[@]}"; do
    i=$((i + 1))
    [ "$i" -lt "$START" ] && continue
    IFS='|' read -r name slew sm kp ki kd <<< "$cfg"
    if [ -n "$ONLY" ] && [[ " $ONLY " != *" $name "* ]]; then continue; fi

    echo
    echo "############ $i/${#CONFIGS[@]} — $name: slew=$slew smooth=$sm kp=$kp ki=$ki kd=$kd ############"
    date
    if [ "$DRY_RUN" = "1" ]; then
        echo "  NAME=$name BS_SLEW=$slew BS_PITCH_SMOOTH=$sm BS_PITCH_KP=$kp BS_PITCH_KI=$ki BS_PITCH_KD=$kd bash $RUN"
        continue
    fi

    NAME="$name" BS_SLEW="$slew" BS_PITCH_SMOOTH="$sm" BS_PITCH_KP="$kp" \
        BS_PITCH_KI="$ki" BS_PITCH_KD="$kd" bash "$RUN" \
        || echo "⚠️ прогон $name завершился с ошибкой — продолжаю свип"

    # метрика по свежему bag'у (стек после прогона поднят — идём в него;
    # если лежит, поднимаем тот же образ разово, без устройств)
    if docker exec p1317_nav true 2>/dev/null; then
        docker exec -e BC_BAG="/root/sim_ws/output/${name}_bag" -e BC_CSV=/root/sim_ws/output/lag_sweep.csv \
            -e BC_NAME="$name" -e BC_SLEW="$slew" -e BC_TABLE=0 p1317_nav \
            bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/back_check.py' \
            | tail -25 || echo "⚠️ разбор $name не удался"
    else
        docker run --rm -v "$OUT:/root/sim_ws/output" -v "$SCRIPT_DIR:/lab:ro" \
            -e BC_BAG="/root/sim_ws/output/${name}_bag" -e BC_CSV=/root/sim_ws/output/lag_sweep.csv \
            -e BC_NAME="$name" -e BC_SLEW="$slew" -e BC_TABLE=0 sim-nav:latest \
            bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/back_check.py' \
            | tail -25 || echo "⚠️ разбор $name не удался"
    fi
done

echo
echo "############ СВИП ЗАВЕРШЁН ############"
date
column -s, -t "$CSV" 2>/dev/null || cat "$CSV" 2>/dev/null
