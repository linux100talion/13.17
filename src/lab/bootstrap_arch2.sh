#!/usr/bin/env bash
# Команда `bootstrap_arch2` — СРЕЗ 1 рефактора управления (ветка nn2_c3_control):
# взлёт в ALT_HOLD + gz-position-hold + боковой/продольный ЧЕЛНОК на новом
# hexagonal-ядре (control_pkg + mission_pkg). Запускает ноду
# `ros2 run mission_pkg bootstrap_arch2` (а не монолит /lab/alt_hold_bootstrap.py).
#
# Поведенчески воспроизводит liftland --gz-hold --gz-shuttle-* монолита; закон
# управления сверен ПОБИТОВО оффлайн-тестом src/control/test/test_gz_shuttle_equiv.py.
# Назначение — валидация среза 1 в симе рядом с монолитом (одинаковая метрика).
#
# Запуск внутри nav:  docker exec p1317_nav bash /lab/bootstrap_arch2.sh
# В секвенсоре:       bash src/lab/capture_scene.sh 960x540 bootstrap_arch2
#
# ОРТОГОНАЛЬНЫЙ путь (профиль-миссии) — рекомендуемый:
#   BS_STAB     — стабилизатор(ы): GzPosHold | DpRollHold+DpYawHold | DpHold | VinsHold | manual
#   BS_MISSION  — плейлист: Mission1 | square | bootstrap | 'climb3,mv_fwd2,mv_bkwd4,landing3'
#   BS_MV_LEVEL — уровень стика mv_* сегментов (0.3)
#   Пример: BS_STAB=GzPosHold BS_MISSION="climb3,mv_fwd2,mv_bkwd4,landing3" bash /lab/bootstrap_arch2.sh
#
# Параметры через env (подмножество BS_*, совместимое с liftland.sh):
#   BS_PILOT (scripted)         — источник стиков: scripted | joy (живой TX12 по USB,
#                                 поднимает joy_linux_node; устройство — JOY_DEV,
#                                 default /dev/input/js0) | ros (ЛЕГАСИ, петля) |
#                                 replay (виртуальный пилот joy_replay.py по сценарию:
#                                 BS_REPLAY_SCENARIO/BS_REPLAY_RAW, см. src/lab/joystick/)
#   BS_ALT (3)                  — целевая высота, м
#   BS_GZ_KP/KD/KI/IMAX/MAX     — гейны gz-hold (дефолты ноды 40/120/8/100/150)
#   BS_GZ_PSIGN/RSIGN           — знаки коррекции (±1)
#   BS_GZ_SHUTTLE_LEVEL (0.3)/LEG (3)/PAUSE (2) — челнок: уровень стика [-1..1] на плече,
#                                 длительность плеча и пауза, sim-сек;
#                                 BS_GZ_SHUTTLE_FWD=1 — продольный
#   BS_THROTTLE_CLIMB           — газ подъёма, PWM
#   BS_MODE/ARM/CLIMB/LAND_BUDGET — бюджеты фаз, sim-сек
set -e
source /opt/ros/humble/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

if ! ros2 pkg list 2>/dev/null | grep -q '^mission_pkg$'; then
    echo "  ОШИБКА: mission_pkg не собран. Проверь mounts в docker-compose.yml и"
    echo "  сборку в nav_up.sh (colcon build --packages-select control_pkg mission_pkg)."
    echo "  Первый запуск после добавления mounts требует: make fresh-start."
    exit 1
fi

# ЖИВОЙ ПУЛЬТ (BS_PILOT=joy): TX12 в режиме USB-джойстика → joy_linux_node → /joy →
# JoyPilot. Мимо FCU — под активным override /mavros/rc/in отдаёт эхо собственной
# команды ноды (петля), поэтому rc/in для живых стиков НЕ используется.
# Драйвер живёт только на время прогона (trap ниже). Устройство — JOY_DEV
# (default /dev/input/js0; проброшен каталогом, hotplug работает).
JOY_PID=""
if [ "${BS_PILOT:-}" = "joy" ]; then
    # BS_JOY_DEV — синоним под автопроброс env в capture_scene.sh (маска BS_*)
    JOY_DEV="${BS_JOY_DEV:-${JOY_DEV:-/dev/input/js0}}"
    if [ ! -e "$JOY_DEV" ]; then
        echo "  ОШИБКА: BS_PILOT=joy, но $JOY_DEV нет. Пульт в USB-режиме воткнут?"
        echo "  Проверка на хосте: ls /dev/input/js*; в контейнере: jstest $JOY_DEV"
        exit 1
    fi
    # default_trig_val — публиковать НАЧАЛЬНОЕ состояние осей (JS_EVENT_INIT):
    # без него тумблер/стик, выставленный ДО старта ноды, невидим (ось = 0.00 до
    # первого движения) — полёт 2026-08-17: CH6 вверх выставлен до взлёта → нода
    # весь полёт видела «центр» → чистый ALT_HOLD вместо нашего стабилизатора.
    ros2 run joy_linux joy_linux_node --ros-args \
        -p dev:="$JOY_DEV" -p autorepeat_rate:=20.0 -p default_trig_val:=true \
        > /root/sim_ws/output/joy.log 2>&1 &
    JOY_PID=$!
    trap '[ -n "$JOY_PID" ] && kill "$JOY_PID" 2>/dev/null || true' EXIT
    echo ">>> joy_linux_node запущен (dev=$JOY_DEV, pid=$JOY_PID, лог output/joy.log)"
fi

# ВИРТУАЛЬНЫЙ ПИЛОТ (BS_PILOT=replay): joy_replay.py публикует /joy по сценарию
# (src/lab/joystick/, см. README.md там) — нода видит его как живой пульт
# (--pilot joy), весь стек ниже /joy идентичен ручному полёту. Источник:
#   BS_REPLAY_SCENARIO — семантический сценарий .json (боевой режим);
#   BS_REPLAY_RAW      — сырой таймлайн .jsonl из joy_timeline.py (валидация).
# Знаки осей BS_JOY_SIGNS уходят и ноде, и реплею — рассинхрон невозможен.
if [ "${BS_PILOT:-}" = "replay" ]; then
    RARGS=()
    [ -n "${BS_REPLAY_SCENARIO:-}" ] && RARGS+=(--scenario "$BS_REPLAY_SCENARIO")
    [ -n "${BS_REPLAY_RAW:-}" ]      && RARGS+=(--raw "$BS_REPLAY_RAW")
    if [ ${#RARGS[@]} -eq 0 ]; then
        echo "  ОШИБКА: BS_PILOT=replay требует BS_REPLAY_SCENARIO (сценарий .json)"
        echo "  или BS_REPLAY_RAW (сырой .jsonl). См. src/lab/joystick/README.md."
        exit 1
    fi
    for f in "${BS_REPLAY_SCENARIO:-}" "${BS_REPLAY_RAW:-}"; do
        if [ -n "$f" ] && [ ! -e "$f" ]; then
            echo "  ОШИБКА: файл реплея не найден: $f (путь — ВНУТРИ контейнера:"
            echo "  /lab/joystick/scenarios/... или /root/sim_ws/output/joystick/...)"
            exit 1
        fi
    done
    # форма --signs=… обязательна: значение начинается с «-», argparse иначе падает
    [ -n "${BS_JOY_SIGNS:-}" ] && RARGS+=("--signs=$BS_JOY_SIGNS")
    # геозабор ВИРТУАЛЬНОГО ПИЛОТА (freefly фенс не проверяет — «пилот сам
    # страховка», и пилот теперь joy_replay); перекрывает "fence" сценария
    [ -n "${BS_REPLAY_FENCE:-}" ] && RARGS+=(--fence "$BS_REPLAY_FENCE")
    python3 /lab/joystick/joy_replay.py "${RARGS[@]}" \
        > /root/sim_ws/output/joy_replay.log 2>&1 &
    JOY_PID=$!
    trap '[ -n "$JOY_PID" ] && kill "$JOY_PID" 2>/dev/null || true' EXIT
    BS_PILOT=joy          # нода получает --pilot joy: JoyPilot на /joy, как с TX12
    echo ">>> joy_replay запущен (pid=$JOY_PID, лог output/joy_replay.log)"
fi

ARGS=()
# ОРТОГОНАЛЬНЫЙ путь профиль-миссий: стабилизатор (BS_STAB) × плейлист (BS_MISSION).
#   BS_STAB    — GzPosHold | DpRollHold+DpYawHold | DpHold | VinsHold | manual ('' → GzPosHold)
#   BS_MISSION — имя из MISSIONS (Mission1/square/bootstrap) ИЛИ 'climb3,mv_fwd2,mv_bkwd4,landing3'
#   BS_MV_LEVEL— глобальный уровень стика mv_* сегментов (дефолт 0.3)
# Задан BS_MISSION → BS_CONTROL_MODE игнорируется (это легаси-ярлык).
[ -n "${BS_STAB:-}" ]            && ARGS+=(--stab "$BS_STAB")
[ -n "${BS_MISSION:-}" ]         && ARGS+=(--mission "$BS_MISSION")
[ -n "${BS_MV_LEVEL:-}" ]        && ARGS+=(--mv-level "$BS_MV_LEVEL")
# ПРЕДЕЛ СКОРОСТИ ИЗМЕНЕНИЯ команды, PWM/сек. Борт выходит на угол за τ=0.27с (замер по
# ступеням A1/A2); команда шириной в кадр даёт 11% угла, при 100 PWM/с полный ход за 1.5с
# ≈ 5.5τ → 99.6%. Без этого выход PID шёл в провод как есть и знак менялся раньше, чем
# угол устанавливался (факт 3 в src/control/ToDo.md).
[ -n "${BS_SLEW:-}" ]            && ARGS+=(--slew "$BS_SLEW")
# геозабор: увод дальше N метров от старта → сразу land (стендовая страховка, по gt)
[ -n "${BS_FENCE:-}" ]           && ARGS+=(--fence "$BS_FENCE")
[ -n "${BS_ROLL_IMAX:-}" ]       && ARGS+=(--roll-imax "$BS_ROLL_IMAX")
[ -n "${BS_PITCH_IMAX:-}" ]      && ARGS+=(--pitch-imax "$BS_PITCH_IMAX")
# режим управления фазы EXCITE (легаси, срез 2): shuttle (дефолт) | assisted | manual
[ -n "${BS_CONTROL_MODE:-}" ]    && ARGS+=(--control-mode "$BS_CONTROL_MODE")
[ -n "${BS_EXCITE_MAX:-}" ]      && ARGS+=(--excite-max-sec "$BS_EXCITE_MAX")
[ -n "${BS_PILOT:-}" ]           && ARGS+=(--pilot "$BS_PILOT")
[ -n "${BS_JOY_SIGNS:-}" ]       && ARGS+=(--joy-signs "$BS_JOY_SIGNS")
[ -n "${BS_GZ_CMD_GAIN:-}" ]     && ARGS+=(--gz-cmd-gain "$BS_GZ_CMD_GAIN")
# Демпфер по потоку (пре-VINS): ТРИ НЕЗАВИСИМЫЕ ОСИ — roll, pitch, yaw.
# У каждой свой полный набор, ничего не шарится. Старые BS_FLOW_*/BS_YAWH_* убраны:
# BS_FLOW_* правил разом roll+pitch и маскировал, что оси разные.
[ -n "${BS_ROLL_KP:-}" ]         && ARGS+=(--roll-kp "$BS_ROLL_KP")
[ -n "${BS_ROLL_KI:-}" ]         && ARGS+=(--roll-ki "$BS_ROLL_KI")
[ -n "${BS_ROLL_KD:-}" ]         && ARGS+=(--roll-kd "$BS_ROLL_KD")
[ -n "${BS_ROLL_OSIGN:-}" ]      && ARGS+=(--roll-osign "$BS_ROLL_OSIGN")
[ -n "${BS_ROLL_CMD_GAIN:-}" ]   && ARGS+=(--roll-cmd-gain "$BS_ROLL_CMD_GAIN")
[ -n "${BS_ROLL_SMOOTH:-}" ]     && ARGS+=(--roll-smooth "$BS_ROLL_SMOOTH")
[ -n "${BS_ALT_KP:-}" ]          && ARGS+=(--alt-kp "$BS_ALT_KP")
[ -n "${BS_ALT_RATE_MAX:-}" ]    && ARGS+=(--alt-rate-max "$BS_ALT_RATE_MAX")
[ -n "${BS_ALT_TOL:-}" ]         && ARGS+=(--alt-tol "$BS_ALT_TOL")
[ -n "${BS_PITCH_KP:-}" ]        && ARGS+=(--pitch-kp "$BS_PITCH_KP")
[ -n "${BS_PITCH_KI:-}" ]        && ARGS+=(--pitch-ki "$BS_PITCH_KI")
[ -n "${BS_PITCH_KD:-}" ]        && ARGS+=(--pitch-kd "$BS_PITCH_KD")
[ -n "${BS_KF_ALT_MAX:-}" ]     && ARGS+=(--kf-alt-max "$BS_KF_ALT_MAX")
[ -n "${BS_KF_ALT_HOLD:-}" ]    && ARGS+=(--kf-alt-hold "$BS_KF_ALT_HOLD")
[ -n "${BS_YAW_TRANS_FIX:-}" ]  && ARGS+=(--yaw-trans-fix "$BS_YAW_TRANS_FIX")
[ -n "${BS_ATT_EXTRAP:-}" ]    && ARGS+=(--att-extrap "$BS_ATT_EXTRAP")
[ -n "${BS_ATT_INTERP:-}" ]    && ARGS+=(--att-interp "$BS_ATT_INTERP")
[ -n "${BS_ATT_LATENCY:-}" ]   && ARGS+=(--att-latency "$BS_ATT_LATENCY")
[ -n "${BS_ATT_WAIT_MAX:-}" ]  && ARGS+=(--att-wait-max "$BS_ATT_WAIT_MAX")
[ -n "${BS_IPM_MODEL:-}" ]     && ARGS+=(--ipm-model "$BS_IPM_MODEL")
[ -n "${BS_IPM_DEROT:-}" ]     && ARGS+=(--ipm-derot "$BS_IPM_DEROT")
[ -n "${BS_IPM_WZ_TAU:-}" ]    && ARGS+=(--ipm-wz-tau "$BS_IPM_WZ_TAU")
[ -n "${BS_IPM_WZ_GATE:-}" ]         && ARGS+=(--ipm-wz-gate "$BS_IPM_WZ_GATE")
[ -n "${BS_IPM_WIN:-}" ]       && ARGS+=(--ipm-win "$BS_IPM_WIN")
# адаптивная полоса IPM: 0 = статичная 3-6 м (потолок ~5.85 м), >0 = запас к границе видимости
[ -n "${BS_IPM_ADAPT:-}" ]     && ARGS+=(--ipm-adapt "$BS_IPM_ADAPT")
# комплементарный фильтр скорости IPM (прогноз наклоном тяги + коррекция МНК), сек; 0 = выкл
[ -n "${BS_IPM_VEL_TAU:-}" ]   && ARGS+=(--ipm-vel-tau "$BS_IPM_VEL_TAU")
# пол высоты перцепции для геометрии IPM, м (гейт земли 0.15); 0 = старый гейт 0.5
[ -n "${BS_IPM_ALT_FLOOR:-}" ] && ARGS+=(--ipm-alt-floor "$BS_IPM_ALT_FLOOR")
[ -n "${BS_IPM_SCALE_REF:-}" ] && ARGS+=(--ipm-scale-ref "$BS_IPM_SCALE_REF")
# ФВЧ прогноза ускорения (сек, 0 = выкл): снимает балансирующий ветер наклон, из-за
# которого боковая ось видела −0.25 м/с и борт ехал ровно с этой скоростью
[ -n "${BS_IPM_ACC_TAU:-}" ]   && ARGS+=(--ipm-acc-tau "$BS_IPM_ACC_TAU")
# скорость IPM в EKF (VISION_SPEED_ESTIMATE): 1 = вкл; лечит A4-рампу в корне
[ -n "${BS_VISION_VEL:-}" ]    && ARGS+=(--vision-vel "$BS_VISION_VEL")
[ -n "${BS_VISION_POSE_SRC:-}" ] && ARGS+=(--vision-pose-src "$BS_VISION_POSE_SRC")
[ -n "${BS_GPS_DISABLE:-}" ]     && ARGS+=(--gps-disable "$BS_GPS_DISABLE")
# GPS отсутствует С БУТА (LV=2): очередь EK3 без GPS-веток + мост нулевой позы с земли
[ -n "${BS_GPS_DENIED:-}" ]      && ARGS+=(--gps-denied "$BS_GPS_DENIED")
[ -n "${BS_ALT_SRC:-}" ]         && ARGS+=(--alt-src "$BS_ALT_SRC")
# высота ПЕРЦЕПЦИИ (IPM/опора): global | local (EKF z, GPS-denied) | baro (экспер.)
[ -n "${BS_PERC_ALT_SRC:-}" ]    && ARGS+=(--perc-alt-src "$BS_PERC_ALT_SRC")
# ноль высоты перцепции по арму (1 = вкл): чинит смещение EKF local z −0.2..−0.3 м,
# из-за которого гейт земли IPM не открывался на полёте ниже полуметра
[ -n "${BS_PERC_ALT_ZERO:-}" ]   && ARGS+=(--perc-alt-zero "$BS_PERC_ALT_ZERO")
# сброс VINS по арму (1 = вкл): окно инициализации, накопленное за стояние на земле,
# после отрыва не решается за полёт (O(N³) → ODO -- весь полёт, odom_gets_borken)
[ -n "${BS_VINS_RESTART_ARM:-}" ] && ARGS+=(--vins-restart-arm "$BS_VINS_RESTART_ARM")
[ -n "${BS_SET_ORIGIN:-}" ]      && ARGS+=(--set-origin "$BS_SET_ORIGIN")
# координаты origin — примерная РЕАЛЬНАЯ точка старта (EKF строит по ним модель
# магнитного поля WMM; дефолт = дом SITL, с 2026-08-24 Киев — та же точка, что
#                                 начало координат мира Gazebo; боевой борт задаёт свои)
[ -n "${BS_ORIGIN_LAT:-}" ]      && ARGS+=(--origin-lat "$BS_ORIGIN_LAT")
[ -n "${BS_ORIGIN_LON:-}" ]      && ARGS+=(--origin-lon "$BS_ORIGIN_LON")
[ -n "${BS_ORIGIN_ALT:-}" ]      && ARGS+=(--origin-alt "$BS_ORIGIN_ALT")
[ -n "${BS_IPM_MAX_SPEED:-}" ] && ARGS+=(--ipm-max-speed "$BS_IPM_MAX_SPEED")
[ -n "${BS_PITCH_SOFT_ALT:-}" ] && ARGS+=(--pitch-soft-alt "$BS_PITCH_SOFT_ALT")
[ -n "${BS_ROLL_SOFT_ALT:-}" ]  && ARGS+=(--roll-soft-alt "$BS_ROLL_SOFT_ALT")
[ -n "${BS_PITCH_SOFT_NOISE:-}" ] && ARGS+=(--pitch-soft-noise "$BS_PITCH_SOFT_NOISE")
[ -n "${BS_ROLL_SOFT_NOISE:-}" ]  && ARGS+=(--roll-soft-noise "$BS_ROLL_SOFT_NOISE")
[ -n "${BS_IPM_ALT_FWD:-}" ]   && ARGS+=(--ipm-alt-band-fwd "$BS_IPM_ALT_FWD")
[ -n "${BS_IPM_ALT_LAT:-}" ]   && ARGS+=(--ipm-alt-band-lat "$BS_IPM_ALT_LAT")
[ -n "${BS_IPM_ALT_STILL:-}" ] && ARGS+=(--ipm-alt-still "$BS_IPM_ALT_STILL")
[ -n "${BS_IPM_ARM_FRAMES:-}" ] && ARGS+=(--ipm-arm-frames "$BS_IPM_ARM_FRAMES")
[ -n "${BS_KF_SEG_MIN:-}" ]     && ARGS+=(--kf-seg-min-sec "$BS_KF_SEG_MIN")
[ -n "${BS_KF_SEG_FRAC:-}" ]    && ARGS+=(--kf-seg-frac "$BS_KF_SEG_FRAC")
[ -n "${BS_PITCH_RATE_KP:-}" ] && ARGS+=(--pitch-rate-kp "$BS_PITCH_RATE_KP")
[ -n "${BS_ROLL_RATE_KP:-}" ]  && ARGS+=(--roll-rate-kp "$BS_ROLL_RATE_KP")
# ⚠️ Мост обязан идти В ПАРЕ с аргументом в bootstrap_node.py: BS_* без аргумента доезжает
# до .env и меты прогона, но не до ноды — прогон выглядит настроенным, а летит на дефолте.
# Так пропал свип B3s (ki=5 → фактически 0).
[ -n "${BS_ROLL_RATE_KI:-}" ]  && ARGS+=(--roll-rate-ki "$BS_ROLL_RATE_KI")
[ -n "${BS_ROLL_RATE_KD:-}" ]  && ARGS+=(--roll-rate-kd "$BS_ROLL_RATE_KD")
[ -n "${BS_PITCH_RATE_KI:-}" ] && ARGS+=(--pitch-rate-ki "$BS_PITCH_RATE_KI")
[ -n "${BS_ROLL_RATE_KI_TRIM:-}" ]  && ARGS+=(--roll-rate-ki-trim "$BS_ROLL_RATE_KI_TRIM")
[ -n "${BS_PITCH_RATE_KI_TRIM:-}" ] && ARGS+=(--pitch-rate-ki-trim "$BS_PITCH_RATE_KI_TRIM")
[ -n "${BS_PITCH_RATE_KD:-}" ] && ARGS+=(--pitch-rate-kd "$BS_PITCH_RATE_KD")
# cmd_gain rate-осей: стик = целевая скорость демпфера, м/с при полном стике
# (0 = чистое удержание, стики roll/pitch в DpHoldM игнорируются)
[ -n "${BS_ROLL_RATE_CMD_GAIN:-}" ]  && ARGS+=(--roll-rate-cmd-gain "$BS_ROLL_RATE_CMD_GAIN")
[ -n "${BS_PITCH_RATE_CMD_GAIN:-}" ] && ARGS+=(--pitch-rate-cmd-gain "$BS_PITCH_RATE_CMD_GAIN")
# станция-кипинг rate-осей: стик в центре = держать точку (pos_kp 1/с, vmax м/с); 0 = выкл
[ -n "${BS_PITCH_POS_KP:-}" ]   && ARGS+=(--pitch-pos-kp "$BS_PITCH_POS_KP")
[ -n "${BS_PITCH_POS_VMAX:-}" ] && ARGS+=(--pitch-pos-vmax "$BS_PITCH_POS_VMAX")
[ -n "${BS_ROLL_POS_KP:-}" ]    && ARGS+=(--roll-pos-kp "$BS_ROLL_POS_KP")
[ -n "${BS_ROLL_POS_VMAX:-}" ]  && ARGS+=(--roll-pos-vmax "$BS_ROLL_POS_VMAX")
# два закона станции: BRAKE (уходим от точки быстрее 0.3 м/с: цель −brake·v_изм,
# кламп ±brake_vmax) / RETURN (pos_kp/pos_vmax, √-кап acc м/с²); anti-windup И-члена
# rate-осей (1 = вкл). См. config.py, _FlowDamper1D.__init__
[ -n "${BS_ROLL_POS_BRAKE:-}" ]       && ARGS+=(--roll-pos-brake "$BS_ROLL_POS_BRAKE")
[ -n "${BS_ROLL_POS_BRAKE_VMAX:-}" ]  && ARGS+=(--roll-pos-brake-vmax "$BS_ROLL_POS_BRAKE_VMAX")
[ -n "${BS_ROLL_POS_ACC:-}" ]         && ARGS+=(--roll-pos-acc "$BS_ROLL_POS_ACC")
[ -n "${BS_PITCH_POS_BRAKE:-}" ]      && ARGS+=(--pitch-pos-brake "$BS_PITCH_POS_BRAKE")
[ -n "${BS_PITCH_POS_BRAKE_VMAX:-}" ] && ARGS+=(--pitch-pos-brake-vmax "$BS_PITCH_POS_BRAKE_VMAX")
[ -n "${BS_PITCH_POS_ACC:-}" ]        && ARGS+=(--pitch-pos-acc "$BS_PITCH_POS_ACC")
[ -n "${BS_ROLL_POS_BRAKE_V:-}" ]     && ARGS+=(--roll-pos-brake-v "$BS_ROLL_POS_BRAKE_V")
[ -n "${BS_PITCH_POS_BRAKE_V:-}" ]    && ARGS+=(--pitch-pos-brake-v "$BS_PITCH_POS_BRAKE_V")
[ -n "${BS_ROLL_POS_ALT_BAND:-}" ]    && ARGS+=(--roll-pos-alt-band "$BS_ROLL_POS_ALT_BAND")
[ -n "${BS_PITCH_POS_ALT_BAND:-}" ]   && ARGS+=(--pitch-pos-alt-band "$BS_PITCH_POS_ALT_BAND")
[ -n "${BS_RATE_AWU:-}" ]             && ARGS+=(--rate-anti-windup "$BS_RATE_AWU")
[ -n "${BS_STATION_FRAME:-}" ]        && ARGS+=(--station-frame "$BS_STATION_FRAME")
[ -n "${BS_STATION_HEADING:-}" ]      && ARGS+=(--station-heading "$BS_STATION_HEADING")
[ -n "${BS_PITCH_OSIGN:-}" ]     && ARGS+=(--pitch-osign "$BS_PITCH_OSIGN")
[ -n "${BS_PITCH_CMD_GAIN:-}" ]  && ARGS+=(--pitch-cmd-gain "$BS_PITCH_CMD_GAIN")
[ -n "${BS_PITCH_SMOOTH:-}" ]    && ARGS+=(--pitch-smooth "$BS_PITCH_SMOOTH")
[ -n "${BS_YAW_KP:-}" ]          && ARGS+=(--yaw-kp "$BS_YAW_KP")
[ -n "${BS_YAW_KI:-}" ]          && ARGS+=(--yaw-ki "$BS_YAW_KI")
[ -n "${BS_YAW_KD:-}" ]          && ARGS+=(--yaw-kd "$BS_YAW_KD")
[ -n "${BS_YAW_LEAK:-}" ]        && ARGS+=(--yaw-leak "$BS_YAW_LEAK")
[ -n "${BS_YAW_OSIGN:-}" ]       && ARGS+=(--yaw-osign "$BS_YAW_OSIGN")
[ -n "${BS_YAW_CMD_GAIN:-}" ]    && ARGS+=(--yaw-cmd-gain "$BS_YAW_CMD_GAIN")
[ -n "${BS_YAW_SMOOTH:-}" ]      && ARGS+=(--yaw-smooth "$BS_YAW_SMOOTH")
# общий темп рыскания, °/с при полном стике (дефолт 28.65 = темп GzHold)
[ -n "${BS_YAW_RATE_FULL:-}" ]   && ARGS+=(--yaw-rate-full "$BS_YAW_RATE_FULL")
[ -n "${BS_YAW_MAX_RATE:-}" ]    && ARGS+=(--yaw-max-rate "$BS_YAW_MAX_RATE")
[ -n "${BS_YAW_ARM_FRAMES:-}" ]  && ARGS+=(--yaw-arm-frames "$BS_YAW_ARM_FRAMES")
# прямая передача yaw-стика, PWM при полном стике (0 = выкл; лечение пружины курса)
[ -n "${BS_YAW_PILOT_GAIN:-}" ]  && ARGS+=(--yaw-pilot-gain "$BS_YAW_PILOT_GAIN")
[ -n "${BS_YAW_V_GATE:-}" ]      && ARGS+=(--yaw-v-gate "$BS_YAW_V_GATE")
# зрение БЕЗ демпфера: писать /flow_dbg* при Gz*/manual (замер перцепта на заданном движении)
[ "${BS_FLOW_OBS:-0}" = "1" ]    && ARGS+=(--flow-observe)
# рантайм switch Flow→Vins по «VINS ready» (только flow_assist)
[ "${BS_HANDOVER_VINS:-0}" = "1" ] && ARGS+=(--handover-vins)
[ -n "${BS_VINS_MIN:-}" ]        && ARGS+=(--vins-min "$BS_VINS_MIN")
# зрелость VINS для EKF-свапа: sim-СЕКУНД от первой одометрии (гейт по времени
# потока, не по счётчику; пол по счётчику — BS_VINS_MIN)
[ -n "${BS_RIPE_SEC:-}" ]        && ARGS+=(--ripe-sec "$BS_RIPE_SEC")
# 2-я ступень гейта — детектор residual+ratio (ripeness.py; 0 = только время)
[ -n "${BS_RIPE_DET:-}" ]        && ARGS+=(--ripe-det "$BS_RIPE_DET")
# схема «SF-мастер» селектора (BS_SF_MASTER=1): SF (CH7) = мастер сырых стиков,
# SC (CH6) = потолок лесенки демпфер/VinsHold/LOITER (⚠️ не под старые реплеи)
[ -n "${BS_SF_MASTER:-}" ]       && ARGS+=(--sf-master "$BS_SF_MASTER")
# штатный LOITER-на-VINS: freefly-центр CH6 (BS_FF_LOITER=1) и бюджет гейта loiter<t>
[ -n "${BS_FF_LOITER:-}" ]       && ARGS+=(--ff-loiter "$BS_FF_LOITER")
[ -n "${BS_LOITER_ALT:-}" ]      && ARGS+=(--loiter-alt "$BS_LOITER_ALT")
# ярус LOITER: стики = скорость в осях МИРА (TrackHold — yaw вращает нос, не
# траекторию; агро-галсы без крена виража, см. config.loiter_track)
[ -n "${BS_LOITER_TRACK:-}" ]    && ARGS+=(--loiter-track "$BS_LOITER_TRACK")
# ярус LOITER, путь 2: потолок крена виража, ° (YawBankLimit — темп yaw по
# скорости |ω| ≤ g·tan(φ)/v; альтернатива TrackHold, см. config.loiter_bank_max)
[ -n "${BS_LOITER_BANK_MAX:-}" ] && ARGS+=(--loiter-bank-max "$BS_LOITER_BANK_MAX")
# VinsHold: kd на ошибке скорости (лечит звон/долг уставки при полёте по
# прямой, серия eagle 2026-09-02; см. config.vins_kd_err)
[ -n "${BS_VINS_KD_ERR:-}" ]     && ARGS+=(--vins-kd-err "$BS_VINS_KD_ERR")
# VinsHold: защёлка трима — И-член заморожен от живого стика до «гвоздя»
# (аналог _TRIM_LATCH станции; см. config.vins_i_latch)
[ -n "${BS_VINS_I_LATCH:-}" ]    && ARGS+=(--vins-i-latch "$BS_VINS_I_LATCH")
# VinsHold: гвоздь по остановке — на отпускании стика уставка перевязывается
# на точку, где борт встал (как штатный LOITER; см. config.vins_pin_stop)
[ -n "${BS_VINS_PIN_STOP:-}" ]   && ARGS+=(--vins-pin-stop "$BS_VINS_PIN_STOP")
# VinsHold: предиктор позы между 10 Гц отсчётами VINS — мёртвое счисление
# v_vins·возраст, лечит пилу kp·e против бегущей уставки (config.vins_predict)
[ -n "${BS_VINS_PREDICT:-}" ]    && ARGS+=(--vins-predict "$BS_VINS_PREDICT")
# VinsHold: сглаживание vins-скорости для D-члена (ФНЧ τ с) — лечит пилу
# команды от kd·(сырая конечная разность позы) (config.vins_vsmooth)
[ -n "${BS_VINS_VSMOOTH:-}" ]    && ARGS+=(--vins-vsmooth "$BS_VINS_VSMOOTH")
# Ярус 1: выбор стабилизатора VINS (vinshold | dpvins) + гейны DpVins
# (velocity-каскад, плавная замена VinsHold; см. config.dpvins_* / vins_axes.py)
[ -n "${BS_VINS_STAB:-}" ]       && ARGS+=(--vins-stab "$BS_VINS_STAB")
[ -n "${BS_DPVINS_KP_FWD:-}" ]   && ARGS+=(--dpvins-kp-fwd "$BS_DPVINS_KP_FWD")
[ -n "${BS_DPVINS_KP_LAT:-}" ]   && ARGS+=(--dpvins-kp-lat "$BS_DPVINS_KP_LAT")
[ -n "${BS_DPVINS_KI:-}" ]       && ARGS+=(--dpvins-ki "$BS_DPVINS_KI")
[ -n "${BS_DPVINS_CMD_GAIN:-}" ] && ARGS+=(--dpvins-cmd-gain "$BS_DPVINS_CMD_GAIN")
[ -n "${BS_DPVINS_POS_KP:-}" ]   && ARGS+=(--dpvins-pos-kp "$BS_DPVINS_POS_KP")
[ -n "${BS_DPVINS_POS_VMAX:-}" ] && ARGS+=(--dpvins-pos-vmax "$BS_DPVINS_POS_VMAX")
[ -n "${BS_DPVINS_POS_ACC:-}" ]  && ARGS+=(--dpvins-pos-acc "$BS_DPVINS_POS_ACC")
[ -n "${BS_DPVINS_VSMOOTH:-}" ]  && ARGS+=(--dpvins-vsmooth "$BS_DPVINS_VSMOOTH")
# мягкая посадка по кнопке SA в freefly (config.ff_land): гейт «низко и стоим»,
# скорость снижения ветки ALT_HOLD, где кнопка в /joy ('b0' | 'a7' | '')
[ -n "${BS_FF_LAND:-}" ]         && ARGS+=(--ff-land "$BS_FF_LAND")
[ -n "${BS_LAND_ALT_MAX:-}" ]    && ARGS+=(--land-alt-max "$BS_LAND_ALT_MAX")
[ -n "${BS_LAND_V_MAX:-}" ]      && ARGS+=(--land-v-max "$BS_LAND_V_MAX")
[ -n "${BS_LAND_RATE:-}" ]       && ARGS+=(--land-rate "$BS_LAND_RATE")
[ -n "${BS_LAND_JOY+x}" ]        && ARGS+=(--land-joy "$BS_LAND_JOY")
[ -n "${BS_LOITER_GATE_BUDGET:-}" ] && ARGS+=(--loiter-gate-budget "$BS_LOITER_GATE_BUDGET")
[ -n "${BS_EKF_POS_BUDGET:-}" ]  && ARGS+=(--ekf-pos-budget "$BS_EKF_POS_BUDGET")
[ -n "${BS_ALT:-}" ]              && ARGS+=(--alt "$BS_ALT")
[ -n "${BS_GZ_KP:-}" ]           && ARGS+=(--gz-kp "$BS_GZ_KP")
[ -n "${BS_GZ_KD:-}" ]           && ARGS+=(--gz-kd "$BS_GZ_KD")
[ -n "${BS_GZ_KI:-}" ]           && ARGS+=(--gz-ki "$BS_GZ_KI")
[ -n "${BS_GZ_IMAX:-}" ]         && ARGS+=(--gz-imax "$BS_GZ_IMAX")
[ -n "${BS_GZ_MAX:-}" ]          && ARGS+=(--gz-max "$BS_GZ_MAX")
[ -n "${BS_GZ_PSIGN:-}" ]        && ARGS+=(--gz-psign "$BS_GZ_PSIGN")
[ -n "${BS_GZ_RSIGN:-}" ]        && ARGS+=(--gz-rsign "$BS_GZ_RSIGN")
# ⚠️ Были BS_GZ_SHUTTLE_A/V → --gz-shuttle-a/-v: челнок давно переписан с «амплитуда/
# скорость» на «уровень стика/плечо», и этих флагов в ноде НЕТ — argparse на них падал.
# Пересчитать A/V в level/leg нельзя (разные величины), поэтому мёртвые ручки убраны, а
# не переименованы. Нашла их check_knobs.sh на первом же запуске.
[ -n "${BS_GZ_SHUTTLE_LEVEL:-}" ] && ARGS+=(--gz-shuttle-level "$BS_GZ_SHUTTLE_LEVEL")
[ -n "${BS_GZ_SHUTTLE_LEG:-}" ]   && ARGS+=(--gz-shuttle-leg "$BS_GZ_SHUTTLE_LEG")
[ -n "${BS_GZ_SHUTTLE_PAUSE:-}" ] && ARGS+=(--gz-shuttle-pause "$BS_GZ_SHUTTLE_PAUSE")
[ "${BS_GZ_SHUTTLE_FWD:-0}" = "1" ] && ARGS+=(--gz-shuttle-fwd)
[ -n "${BS_THROTTLE_CLIMB:-}" ]  && ARGS+=(--throttle-climb "$BS_THROTTLE_CLIMB")
[ -n "${BS_MODE_BUDGET:-}" ]     && ARGS+=(--mode-budget "$BS_MODE_BUDGET")
[ -n "${BS_ARM_BUDGET:-}" ]      && ARGS+=(--arm-budget "$BS_ARM_BUDGET")
[ -n "${BS_CLIMB_BUDGET:-}" ]    && ARGS+=(--climb-budget "$BS_CLIMB_BUDGET")
[ -n "${BS_LAND_BUDGET:-}" ]     && ARGS+=(--land-budget "$BS_LAND_BUDGET")

echo ">>> ARCH2 bootstrap (gz-hold + shuttle) на control_pkg/mission_pkg: ${ARGS[*]}"
ros2 run mission_pkg bootstrap_arch2 "${ARGS[@]}"
echo ">>> bootstrap_arch2 завершён."
