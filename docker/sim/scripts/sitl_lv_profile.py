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
#   LV=2: «GPS ОТСУТСТВУЕТ С БУТА» (модель боевого борта без приёмника):
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
