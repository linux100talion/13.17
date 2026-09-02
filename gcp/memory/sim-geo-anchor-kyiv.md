---
name: sim-geo-anchor-kyiv
description: Геодезическая привязка сима — три точки (мир, дом SITL, origin ноды), с 2026-08-24 все Киев
metadata:
  type: project
---

Начало координат Gazebo = реальная точка Земли, и она обязана совпадать в ТРЁХ
местах (иначе «PreArm: Check mag field» и арма нет — EK3 строит WMM от origin, а
магнитометр SITL рисуется от дома SITL):

1. `docker/sim/worlds/mili_fortress.sdf` — `<spherical_coordinates>`
   (до 2026-08-24 блока НЕ БЫЛО вовсе, потерян при порте Classic→Harmonic);
2. `docker/sim/scripts/sim_up.sh` — `SIM_HOME` → `sim_vehicle --custom-location`
   (раньше дом брался дефолтом CMAC, Канберра);
3. `src/mission/mission_pkg/config.py` — `origin_lat/lon/alt` (`BS_ORIGIN_*`),
   их нода шлёт `SET_GPS_GLOBAL_ORIGIN` в безжпсном буте LV=2.

Сейчас везде Киев: 50.450100, 30.523400, 180 м AMSL.

**Why:** «условный Киев» при доме-CMAC уже стоил прогона (`lv2_replay_20260824_034433`).

**How to apply:** менять точку — только все три места разом. Связано:
[[spawn-from-landing]], [[lv2-gps-denied]].
