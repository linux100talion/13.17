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
#   BS_ALT (3)                  — целевая высота, м
#   BS_GZ_KP/KD/KI/IMAX/MAX     — гейны gz-hold (дефолты ноды 40/120/8/100/150)
#   BS_GZ_PSIGN/RSIGN           — знаки коррекции (±1)
#   BS_GZ_SHUTTLE_A (5)/V (1.5)/PAUSE (2) — челнок; BS_GZ_SHUTTLE_FWD=1 — продольный
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
# зрение БЕЗ демпфера: писать /flow_dbg* при Gz*/manual (замер перцепта на заданном движении)
[ "${BS_FLOW_OBS:-0}" = "1" ]    && ARGS+=(--flow-observe)
# рантайм switch Flow→Vins по «VINS ready» (только flow_assist)
[ "${BS_HANDOVER_VINS:-0}" = "1" ] && ARGS+=(--handover-vins)
[ -n "${BS_VINS_MIN:-}" ]        && ARGS+=(--vins-min "$BS_VINS_MIN")
[ -n "${BS_ALT:-}" ]              && ARGS+=(--alt "$BS_ALT")
[ -n "${BS_GZ_KP:-}" ]           && ARGS+=(--gz-kp "$BS_GZ_KP")
[ -n "${BS_GZ_KD:-}" ]           && ARGS+=(--gz-kd "$BS_GZ_KD")
[ -n "${BS_GZ_KI:-}" ]           && ARGS+=(--gz-ki "$BS_GZ_KI")
[ -n "${BS_GZ_IMAX:-}" ]         && ARGS+=(--gz-imax "$BS_GZ_IMAX")
[ -n "${BS_GZ_MAX:-}" ]          && ARGS+=(--gz-max "$BS_GZ_MAX")
[ -n "${BS_GZ_PSIGN:-}" ]        && ARGS+=(--gz-psign "$BS_GZ_PSIGN")
[ -n "${BS_GZ_RSIGN:-}" ]        && ARGS+=(--gz-rsign "$BS_GZ_RSIGN")
[ -n "${BS_GZ_SHUTTLE_A:-}" ]     && ARGS+=(--gz-shuttle-a "$BS_GZ_SHUTTLE_A")
[ -n "${BS_GZ_SHUTTLE_V:-}" ]     && ARGS+=(--gz-shuttle-v "$BS_GZ_SHUTTLE_V")
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
