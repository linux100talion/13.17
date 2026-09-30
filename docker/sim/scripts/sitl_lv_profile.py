#!/usr/bin/env python3
# ============================================================================
# sitl_lv_profile.py — подготовка eeprom SITL под freefly-профиль (LV=0/1).
# Запускается ВНУТРИ контейнера simulator (вызывает src/lab/freefly_lv.sh):
#   PYTHONPATH=/root/ardupilot/modules/mavlink python3 /scripts/sitl_lv_profile.py <0|1>
#
# Зачем: эти параметры ЖИВУТ В EEPROM (named volume sitl_eeprom), а eeprom
# СИЛЬНЕЕ --defaults (sitl-extra.parm решает только для параметров, которых в
# eeprom НЕТ — урок 994e471). Поэтому переключение профиля — только param_set
# по pymavlink (tcp:5762, как sitl_accel_cal.py). Записанное значение
# применяется на СЛЕДУЮЩЕМ буте SITL — рестарт стека делает capture_scene.sh
# в начале атомарного прогона, отдельный ребут не нужен.
#
#   LV=1: VISO_TYPE=1 (без него FCU выбрасывает VISION_* до EK3 —
#         «Loiter failed: requires position», урок LV1).
#         Остальное (SIM_GPS1_ENABLE, EK3_SRC1_*) самовосстанавливает очередь
#         bootstrap_node ДО арма — здесь не трогаем.
#   LV=0: VISO_TYPE=0 — с 1 прогон без vision-фида НЕ АРМИТСЯ («Arm: VisOdom:
#         not healthy» — обязательный чек, ARMING_CHECK 0 его НЕ снимает,
#         проверено 2026-08-18 дважды). Плюс возврат GPS-профиля EKF, который
#         LV-полёт оставил в eeprom (POSXY=6, VELXY=0, SIM_GPS1=0): в LV=0
#         vision_vel=0 → очередь ноды не работает и сама НЕ вернёт.
#   LV=2: «GPS ОТСУТСТВУЕТ С БУТА» (модель реального борта без приёмника):
#         приёмник глушится в eeprom ЕЩЁ ДО старта ноды (SIM_GPS1_ENABLE=0)
#         и EKF сразу ставится на extnav-пару (POSXY=6 — позу с земли даёт
#         мост нулевой позы bootstrap_node, gps_denied; VELXY=0 — скорость
#         IPM не фьюзим, урок полёта №5: фантомы в ветре → EKF failsafe).
#         Очередь ноды эти же значения лишь самовосстанавливает и датирует
#         extnav_ready зрелостью VINS — источники тут ставим МЫ, до бута.
# ============================================================================
import os
import sys
import time

from pymavlink import mavutil

CONN = 'tcp:127.0.0.1:5762'   # SERIAL1 SITL (5760 занят mavlink_router'ом)

PROFILE = {
    '1': {'VISO_TYPE': 1.0},
    '0': {'VISO_TYPE': 0.0,
          'SIM_GPS1_ENABLE': 1.0,
          'EK3_SRC1_POSXY': 3.0,
          'EK3_SRC1_VELXY': 3.0},
    '2': {'VISO_TYPE': 1.0,
          'SIM_GPS1_ENABLE': 0.0,
          'EK3_SRC1_POSXY': 6.0,
          'EK3_SRC1_VELXY': 0.0},
}

# ПОТОКИ ТЕЛЕМЕТРИИ FCU В EEPROM — для всех LV. В ArduPilot 4.7+ группа стримов
# SRn_* переименована в MAVn_* с нумерацией С ЕДИНИЦЫ: MAV1_* = первый MAVLink-канал
# = SERIAL0 (tcp 5760, mavlink_router → MAVROS); «SR0_RAW_SENS» прошивка не знает
# (нет эха PARAM_VALUE — проверено 2026-09-07). ArduPilot не шлёт RAW_IMU, ATTITUDE,
# LOCAL_POSITION_NED, пока темпы нули; REQUEST_DATA_STREAM из nav_up.sh прошивка
# сама СОХРАНЯЕТ в eeprom (persist_streamrates), так что после первого удачного
# цикла значения уже там — здесь они закрепляются явно и восстанавливаются после
# `make clean`/нового бокса. Прогон 122716 (2026-09-07): MAVROS 200 с без единого
# потока при живом мосте позы, узел «ждал EKF» (ekf=0 = молчал MAVROS); первопричина
# на стороне FCU не установлена — потоки, «готово» после потоков и сторож в ноде =
# три независимые страховки (WIND 168 — только SET_MESSAGE_INTERVAL, в MAV нет).
# Значения = запросам nav_up.sh (те же темпы, что летали все серии): RAW_SENS 200
# (RAW_IMU, SCALED_PRESSURE, GPS_RAW), POSITION 25 (LOCAL_POSITION_NED,
# GLOBAL_POSITION_INT), EXTRA1 50 (ATTITUDE), EXT_STAT 2 (SYS_STATUS,
# EXTENDED_SYS_STATE), EXTRA2 5 (VFR_HUD).
STREAMS = {'MAV1_RAW_SENS': 200.0, 'MAV1_POSITION': 25.0, 'MAV1_EXTRA1': 50.0,
           'MAV1_EXT_STAT': 2.0, 'MAV1_EXTRA2': 5.0,
           # RC_CHANNELS → /mavros/rc/in: что пришло в полётник с RC-входа (мост /joy →
           # RC SITL, src/sim/joy_rc_bridge.py) или из override ноды; 2026-09-30
           'MAV1_RC_CHAN': 25.0}

# РЕВЕРС КАНАЛОВ ПОЛЁТНИКА — сток (0) в КАЖДОМ прогоне. cmd/rc_rev ставит бортовой
# RC2_REVERSED 1 через BS_FCU_PARAMS (идёт последним и перекрывает), а eeprom переживает
# прогоны: без явного возврата реверс унаследовал бы любой соседний кандидат, чей
# профиль BS_FCU_PARAMS про реверс не знает (loiter/rth*, smart_rth). В симе реверс
# не нужен — причуду TX12 по USB исправляет JOY_SIGNS ноды (laptop_move.md §5.7).
RC_REVERSE = {f'RC{i}_REVERSED': 0.0 for i in range(1, 5)}

# ДАЛЬНОМЕР ВЫКЛ — в КАЖДОМ прогоне. Стоковый gazebo-iris.parm (--defaults в sim_up.sh)
# включает RNGFND1_TYPE 1: аналоговый дальномер, который симулирует САМ SITL, а не мир
# Gazebo. Он читает 23–25 м при истинных 1–3 м (статус «исправен»), и ALT_HOLD со
# слежением за поверхностью (SURFTRAK_MODE 1 по умолчанию) держит это фантомное
# расстояние: на плече вперёд показание росло 24.7→25.4 м, и полётник при газе ровно 1500
# сам заказал снижение до земли (vins_init_20260930_120758, dataflash CTUN SAlt/DSAlt).
# В eeprom, а не в sitl-extra.parm: eeprom сильнее --defaults (урок VISO_TYPE).
RANGEFINDER = {'RNGFND1_TYPE': 0.0}

# ОТКЛИК НА УГОЛ — КАК НА РЕАЛЬНОМ БОРТУ (distro/doc/HW/Ardupilot_Params/MP/stellar_cld.txt),
# в КАЖДОМ прогоне (2026-09-30, переезд на углы тюним в симе — laptop_move.md §5.7).
# Гейны в единицах «м/с → угол» переносятся на борт как есть, только если рама в симе
# отвечает на заказанный угол так же: потолок наклона 20° (сток сима 30°), сглаживание
# входа 0.3 (сток 0.1 — втрое мягче), предел темпа крена/тангажа 60 °/с (сток 0 = без
# предела). Пилотские настройки борта (THR_DZ 200, PILOT_SPD 0.5, PILOT_Y_RATE 90, LOIT_*,
# ход 988–2011, мёртвая зона каналов 20) НЕ переносим: газ 0.3 реплеев попал бы в мёртвую
# зону газа. ⚠️ Меняет базовый стек: демпфер получает ×0.67 авторитета по углу (гейны сима
# стояли под 30°) — прогоны не сравнимы бит-в-бит с сериями до 2026-09-30.
BOARD_ATTITUDE = {'ATC_ANGLE_MAX': 20.0, 'ATC_INPUT_TC': 0.3,
                  'ATC_RATE_R_MAX': 60.0, 'ATC_RATE_P_MAX': 60.0}


def read_param(m, name, budget=8.0):
    t0 = time.time()
    while time.time() - t0 < budget:
        m.mav.param_request_read_send(
            m.target_system, m.target_component, name.encode(), -1)
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg is not None and msg.param_id.rstrip('\x00') == name:
            return msg.param_value
    return None


def set_param(m, name, val, tries=5):
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), val,
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.5)
        cur = read_param(m, name)   # эхо перепроверяем ЧТЕНИЕМ (echo флапает)
        if cur is not None and abs(cur - val) < 1e-4:
            return True
        time.sleep(1)
    return False


def main():
    lv = sys.argv[1] if len(sys.argv) > 1 else '1'
    want = PROFILE.get(lv)
    if want is None:
        print(f"ОШИБКА: LV={lv} (ожидаю 0, 1 или 2)")
        return 2
    want = dict(want)     # копия: не мутируем PROFILE между вызовами
    want.update(STREAMS)  # потоки телеметрии — всегда (см. STREAMS)
    want.update(RC_REVERSE)  # реверс каналов — сток, пока BS_FCU_PARAMS не скажет иначе
    want.update(RANGEFINDER)  # фантомный дальномер SITL — выкл (см. RANGEFINDER)
    want.update(BOARD_ATTITUDE)  # отклик на угол как на борту (см. BOARD_ATTITUDE)
    # BS_EKF_DRAG — drag-фьюжн ветра EKF3 (BCOEF, кг/м²; 0 = выкл). Наблюдаемость
    # на VINS-external-nav доказана Ф0 (ветер сошёлся к истине 10 м/с). Даёт
    # стрелку ветра HUD во ВСЕХ режимах (windspeed.md). ⚠️ МЕНЯЕТ EKF (добавляет
    # drag-измерение) — только для ветра HUD, но проверять качество удержания
    # A/B прежде чем доверять для управления. Значение 32 — замер сим-драга на
    # 5-10 м/с. Стрелку рисует только при непустом WIND-стриме (nav_up).
    bcoef = float(os.environ.get('BS_EKF_DRAG', '32') or '0')
    if bcoef > 0 and lv == '2':
        want['EK3_DRAG_BCOEF_X'] = bcoef
        want['EK3_DRAG_BCOEF_Y'] = bcoef
        want['EK3_DRAG_MCOEF'] = 0.0
    elif lv == '2':
        want['EK3_DRAG_BCOEF_X'] = 0.0   # выкл явно (мог остаться в eeprom)
        want['EK3_DRAG_BCOEF_Y'] = 0.0
    # BS_FCU_PARAMS — ПАРАМЕТРЫ FCU ПРОГОНА, «NAME=VALUE» через пробел (профиль
    # loiter/, ключ EXTRA_KEYS). Для случаев, когда поведением командует полётник, а
    # не нода: возврат домой RTL (cmd/rth: RTL_ALT_M, RTL_LOIT_TIME, LAND_ALT_LOW_M).
    # Идут ПОСЛЕДНИМИ — перекрывают профиль LV и потоки, если имена совпали. В
    # loiter/baseline.txt стоят СТОКОВЫЕ значения: eeprom переживает прогоны, и без
    # явного восстановления кандидат отравил бы соседние полёты.
    for tok in (os.environ.get('BS_FCU_PARAMS', '') or '').split():
        name, _, val = tok.partition('=')
        try:
            want[name.strip()] = float(val)
        except ValueError:
            print(f"  ОШИБКА: BS_FCU_PARAMS: не разбирается {tok!r} (жду NAME=ЧИСЛО)")
            return 2
    print(f"  eeprom-профиль LV={lv}: подключаюсь к SITL ({CONN})...", flush=True)
    try:
        m = mavutil.mavlink_connection(CONN, source_system=253)
        m.wait_heartbeat(timeout=25)
    except Exception as e:
        print(f"  ОШИБКА: SITL недоступен ({e})")
        return 1
    rc = 0
    for name, val in want.items():
        cur = read_param(m, name)
        if cur is not None and abs(cur - val) < 1e-4:
            print(f"    {name} = {cur:g} — уже ок", flush=True)
            continue
        if set_param(m, name, val):
            was = '?' if cur is None else f'{cur:g}'
            print(f"    {name}: {was} → {val:g} ✓ (eeprom; применится на "
                  f"рестарте стека)", flush=True)
        else:
            print(f"    ОШИБКА: {name} не установился (нет эха PARAM_VALUE)")
            rc = 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
