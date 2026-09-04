---
name: sim-wind-gusts
description: "Порывы ветра в симе: WIND_GUST → wind_gust.py публикует в runtime-топик WindEffects, расписание в абсолютном sim-времени; DpHold/LOITER держат, DpVins чуть хуже (разбор открыт)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 790e40a9-7511-4c26-b8ed-594eb205c6fa
  modified: 2026-09-04T10:25:05.842Z
---

Порывы поверх постоянного WIND_SPD (2026-09-04, ветка nn2_c3_laptop_wind,
коммиты c2666f7 + ac509c5): плагин WindEffects слушает runtime-топик
`/world/<мир>/wind` (gz.msgs.Wind) — вектор меняется на лету, мир не трогается.
Публикатор `src/lab/wind_gust.py` (живёт в контейнере simulator — биндинги
gz.transport13/msgs10 только там; сам себя туда docker cp): профиль «1−cos
фронт → плато → спад», расписание в АБСОЛЮТНОМ sim-времени (t=0 = старт
Gazebo) → у двух прогонов порывы в одни и те же sim-секунды, честный A/B.
Спека: `WIND_GUST="spd=12 at=60 rise=2 hold=5 fall=4 every=30"` (шапка
скрипта); запускает capture_scene.sh после рестарта, гасит trap; лог фаз
с sim_t — output/wind_gust.log, едет в архив прогона (джойн с bag по
sim-времени). sim_up.sh грузит плагин и на штиле (WIND_SPD=0 + WIND_GUST).

**Why:** порывы = стресс-тест доучивания трима (ki) и гейта «физики висения»
(vins_hover_v=3.0 — честный разгон порывом на центральных стиках может
уронить ярус). Сила квадратична по скорости ветра (WIND_FACTOR не трогать,
см. sim_up.sh): порыв 5→12 ≈ ×5.8 силы.

**How to apply:** `WIND_GUST="spd=10 at=90 rise=2 hold=5 fall=4 every=30"
bash src/lab/freefly_lv.sh`. Полёт 2026-09-04 (spd=10 every=30 поверх базы 5):
DpHold и LOITER-на-VINS держат порывы отлично (первый взгляд), **DpVins чуть
хуже — разбор открыт** (кандидаты: доучивание ki=6 после смены ветра, гвоздь
станции против сдвига равновесия). См. [[dpvins-wind-trim-learn]],
[[damper-low-alt]].

Попутный фикс ac509c5: `make wait` теперь `docker logs --since StartedAt` —
раньше после restart-all проходил по «nav: готово» прошлой жизни контейнера
(логи переживают stop/start), секвенсор стартовал на полуподнятом стеке.
