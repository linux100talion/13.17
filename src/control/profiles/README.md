# Профили настроек — ЕДИНСТВЕННЫЙ источник ручек прогона

Снимки ручек `BS_*` ноды `bootstrap_arch2` по ярусам лесенки и по слоям миссии, чтобы
A/B-кампании шли ОТ ИМЕНОВАННОГО эталона, а у КАЖДОГО поля ноды было ровно одно место.
С 2026-09-07 профили собирает `load.py` (include + дельта), в них лежат и ручки миссии/пилота,
и легаси-поля; дефолты `freefly_lv.sh`/`.env`/ноды под `cmd/*` полностью затенены
(следующий этап — нода без дефолтов: нет ключа → ошибка). Каталоги:

| каталог | что внутри |
|---|---|
| `dphold/` | ярус 0 — демпфер `DpHoldM` на IPM-канале: rate-оси, станция, мягкость, перцепция IPM/углы, курс `DpYawHold` |
| `dpvins/` | ярус 1 — ГЕЙНЫ `DpVins` (velocity-каскад, позиционный контур, трим, BRAKE) |
| `vinshold/` | ярус 1 — ГЕЙНЫ И ФЛАГИ `VinsHold` (откат: 2D-PID, флаги eagle, Gz-hold). Грузится ВСЕГДА рядом с `dpvins/` — какой стабилизатор активен, решает селектор в `vins/` |
| `vins/` | ярусы 1 и 2 — опора VINS: СЕЛЕКТОР `BS_VINS_STAB` (dpvins \| vinshold, откат — `vins/vinshold.txt`), общая защёлка трима, хэндовер и зрелость, гейт здоровья (три канала), мост VINS→EKF |
| `loiter/` | ярус 2 — штатный LOITER на EKF-от-VINS: гейты миссии, TrackHold/YawBankLimit, GPS-denied/EKF, справочно SITL |
| `wind/` | ярусы 0 и 1 — сквозной ветровой трим `WindTrim` (`BS_WIND_TRIM`, серия/порог устойчивого hold) |
| `mission/` | миссия и пилот: `BS_PILOT`/`BS_MISSION`, SF-мастер, кнопка SA, высота и бюджеты фаз, геозабор, мягкая посадка, скриптовые миссии; `replay.txt` — реплей пульта |
| `legacy/` | поля ноды ВНЕ активного стека (control_mode/gz-shuttle, DpRollHold/DpPitchHold, старый flow-путь, KF-высота) значениями = дефолты ноды. Кандидат на вычистку из кода вместе с файлом |

Активный стек (`cmd/bl/bl.sh`, `check.sh` без аргументов):
`dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/guard wind/trim
mission/baseline legacy/baseline` → 190 ключей. `baseline.txt` каждого каталога — ЭТАЛОН на
дату в шапке; кандидат — `include baseline.txt` + изменённые строки с говорящим именем и
гипотезой в шапке (`dpvins/ki30.txt`).

## Формат

Файл — строки `KEY=VALUE` (без пробелов вокруг `=`), комментарии `#`, пустые строки и
директива **`include <файл>`** (путь относительно каталога самого профиля):

- `include baseline.txt` ставит ВСЕ ключи эталона, строки ниже их ПЕРЕКРЫВАЮТ — кандидат =
  эталон + дельта. Новая ручка добавляется в один `baseline.txt`, кандидаты наследуют.
- Между РАЗНЫМИ перечисленными профилями один ключ — ОШИБКА загрузчика (`dphold/` и
  `dpvins/` не спорят); один ключ живёт в одном каталоге.
- Пустое значение (`BS_KF_ALT_HOLD=`) легально: `bootstrap_arch2.sh` пропускает пустые,
  нода берёт `None`. До этапа «нода без дефолтов» это способ сказать «выкл».
- Любая другая строка — ошибка с файлом и номером. Профили НЕ для прямого `source` в
  bash: строка `include` там упадёт («command not found») — намеренно, тихого чтения
  половины файла быть не должно.

Слои значений помечены `# слой:` над блоками — откуда значение пришло ДО 2026-09-07
(история, не приоритет):
- **env** — задавал `src/lab/freefly_lv.sh` или `docker/sim/.env`;
- **нода** — дефолт `mission_pkg/config.py`, в профиле записан ЯВНО тем же значением;
- **SITL** — параметры прошивки (`docker/sim/config/sitl-extra.parm`,
  `docker/sim/scripts/sitl_lv_profile.py`): СПРАВОЧНО, закомментированы (`#SITL …`),
  применяются не через env.

**Не параметры, а аргументы прогона** — в профилях не живут: `BS_REPLAY_SCENARIO`,
`BS_REPLAY_RAW`, `BS_REPLAY_FENCE` (сценарий реплея и его забор — в команде запуска).
Машинное (`VINS_SRC`, `CUDA_ARCH_BIN`, `WORLD`, `LV`) — compose/`docker/sim/.env`
(`docker/sim/env.md`); ветер `WIND_*` — плагин Gazebo, ставит `cmd/<имя>.sh` через `${X:-}`.

## Как применить

```bash
# cmd/<имя>/<имя>.sh (образец — cmd/bl/bl.sh):
P=(dphold/baseline dpvins/ki30 vinshold/baseline vins/baseline loiter/baseline wind/baseline
   mission/baseline legacy/baseline)
set -a
eval "$(python3 src/control/profiles/load.py "${P[@]}")"
set +a
WIND_SPD=1 bash src/lab/freefly_lv.sh
```

`load.py` печатает `export KEY='VALUE'`; `--format plain|json`, `--origin` (откуда каждый
ключ). Профиль перекрывает внешний env и `.env` для СВОИХ ключей (лесенка —
`docker/sim/env.md`); хочешь другой kp — файл-кандидат + копия `cmd/bl` в `cmd/<имя>/`.
Реплей: `BS_PILOT=replay BS_REPLAY_SCENARIO=… bash cmd/bl/bl.sh` — `bl.sh` подставит
`mission/replay`. Откат яруса 1 на VinsHold — `vins/vinshold` вместо `vins/<…>`.

`<RUN>.env` прогона (мета `freefly_lv.sh`) фиксирует, что реально доехало до env.
Сверка: `bash src/control/profiles/check.sh <RUN>.env [профили…]` = `load.py --diff` —
печатает расходящиеся ключи (код 1), считает ключи только в профилях (до этапа «нода без
дефолтов» они в мету не попадали) и `BS_`-ключи меты вне профилей (должно быть 0).
Тест загрузчика и инвариантов профилей репо: `python3 src/control/profiles/test_load.py`.

## Доказательство 2026-09-07 (этап 1–2 рефакторинга параметров)

Стек `cmd/bl` до/после: 134 прежних ключа бит в бит; 56 новых (`mission/`, `legacy/`,
`vinshold/` в стеке, `BS_YAW_CMD_GAIN=`) равны тому, чем летали (мета
`lv2_joy_20260906_231055.env`: расхождений 0, `BS_`-ключей меты вне профилей 0). Каждый
кандидат после конверсии на `include` разрешается в свой прежний набор; унаследованные
ключи (в старых кандидатах их не было) равны дефолтам ноды.

## Что осталось (этап 3: нода без дефолтов)

- 15 полей `BootstrapConfig` без проводки `BS_*` (только дефолт в коде): `alt_dz` 100,
  `alt_span` 400, `alt_rate_full` 3.16, `roll_max` 150, `roll_conf_min` 0.05,
  `roll_conf_full` 0.2, `pitch_max` 150, `pitch_conf_min` 0.05, `pitch_conf_full` 0.2,
  `yaw_imax` 200, `yaw_max` 150, `yaw_conf_min` 0.05, `yaw_conf_full` 0.2,
  `yaw_flow_scale` 0.324, `yaw_settle` 6. Появятся в профилях вместе с `from_env()`.
- Удалить argparse (196 аргументов) и проводку `BS_FOO → --foo` в `bootstrap_arch2.sh`
  (192 строки), автопроброс `-e BS_*` в `capture_scene.sh`; `BootstrapConfig` без
  дефолтов + `from_env()`/`from_profiles()`; тесты и стенды (`test_bootstrap_fsm`,
  `test_pilot_fsm`, `test_mission_plan`, `ipm_video.py`, `ipm_band_ab.py`) — на профили.
- Вычистить `legacy/` из кода или оставить осознанно.

## Что сравнивать

Полёты по ярусам под порывами меряет `src/lab/gust_hold_compare.py` (пик за порыв =
жёсткость, остаток за цикл = якорь); гейт здоровья — `vins_sane_replay.py`. Итог эталона
2026-09-05: DpHold пик 2.4–2.8 м, DpVins 6.3–9.5, остаток за цикл DpHold 0.8–1.5 (копится
по ветру), DpVins 0.1–0.4 (возвращается) — память `dphold-vs-dpvins-gusts`.
