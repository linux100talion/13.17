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
