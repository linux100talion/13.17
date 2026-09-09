#!/usr/bin/env python3
"""Режимы FCU, которых НЕ ЗНАЕТ MAVROS: таблица ArduCopter в нём осталась в 2016-м.

Разбор полёта lv2_joy_20260907_200909 («smart_rth не включился»): запрос до полётника
НЕ ДОЕХАЛ ВООБЩЕ — упёрся в строковую таблицу MAVROS:

    [ERROR] [uas]: MODE: Unknown mode: SMART_RTL
    [INFO]  [uas]: MODE: Known modes are: GUIDED_NOGPS AVOID_ADSB THROW BRAKE POSHOLD
            AUTOTUNE FLIP STABILIZE SPORT ACRO ALT_HOLD AUTO GUIDED LOITER RTL CIRCLE
            POSITION LAND OF_LOITER DRIFT

SMART_RTL появился в ArduCopter уже после того, как этот список в MAVROS перестали
обновлять (в самой libmavros строка есть — но в таблице Rover, у Copter её нет).

Лечение — числом (проверено на живом SITL 2026-09-07): MAVROS принимает НОМЕР режима
строкой (`custom_mode: '21'`), молча передаёт его в SET_MODE, и полётник режим берёт
(/mavros/state показал `CMODE(21)`). Обратно имени он тоже не знает, поэтому в статусе
режим приходит как `CMODE(<n>)` — сравнивать надо с ЛЮБЫМ из двух написаний.

Отсюда две функции: `to_fcu` (что отправить) и `matches` (как узнать в /mavros/state).
Расширять по мере надобности: FLOWHOLD 22, FOLLOW 23, ZIGZAG 24, SYSTEMID 25,
AUTOROTATE 26, AUTO_RTL 27 — MAVROS не знает их все.
"""

# имя режима ArduCopter → номер custom_mode (только те, которых нет у MAVROS)
NUMERIC = {
    'SMART_RTL': 21,
}


def to_fcu(mode: str) -> str:
    """Имя режима → строка для MAVROS: номер, если имени он не знает."""
    n = NUMERIC.get(mode)
    return mode if n is None else str(n)


def names(mode: str) -> tuple:
    """Как этот режим может выглядеть в /mavros/state: имя и/или 'CMODE(n)'."""
    n = NUMERIC.get(mode)
    return (mode,) if n is None else (mode, f'CMODE({n})')


def matches(state_mode, mode) -> bool:
    """Тот ли это режим — с учётом безымянного 'CMODE(n)' от MAVROS."""
    return state_mode in names(mode)


# Режимы, в которых ПОЛОЖЕНИЕ ВЕДЁТ САМ ПОЛЁТНИК по своей навигации, а не пилот
# стиками: наш стек в них ПУСТ и стики стоят в центре ПО ПОСТРОЕНИЮ (шаг Rth), да и
# FCU их всё равно игнорирует. Значит «стик в центре» здесь НЕ признак висения.
# Урок полёта lv2_joy_20260909_044105: гейт здоровья VINS принял центр стиков за
# висение, честные 3-4 м/с возврата — за разнос VINS, закрыл мост vision_pose (brg=0
# дважды, brw=ext), EKF без единственного источника подтяжки уехал на 99 м, и
# полётник сначала сбросил SMART_RTL в RTL («bad position»), а на втором заходе ушёл
# в LAND по EKF-failsafe. LOITER/ALT_HOLD сюда НЕ входят: там стик — наша команда,
# и центр действительно означает «стоим».
NAVIGATED = ('RTL', 'SMART_RTL', 'AUTO', 'GUIDED')


def navigates(state_mode) -> bool:
    """Полётник ведёт борт сам (RTL/SMART_RTL/AUTO/GUIDED)? Имя — из /mavros/state,
    в т.ч. безымянное 'CMODE(n)'. Пустая строка/None = нет (не знаем — считаем наш)."""
    return bool(state_mode) and any(matches(state_mode, m) for m in NAVIGATED)
