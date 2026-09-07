# Профили настроек — ЕДИНСТВЕННЫЙ источник ручек прогона

С 2026-09-07 у лётной ноды `bootstrap_arch2` НЕТ дефолтов: `BootstrapConfig`
(mission_pkg/config.py) — 211 полей без значений, значения только здесь. Схема ключей
= поля датакласса (`BS_<ПОЛЕ>`), второго списка нет. Цепочка:

```
cmd/<имя>/<имя>.sh   держит СПИСОК профилей →  export PROFILES="… world/…"
freefly_lv.sh (хост) load.py $PROFILES → env: мета <RUN>.env, eeprom (BS_EKF_DRAG), ветер (WIND_*)
capture_scene.sh     в контейнер едет только PROFILES (+ BS_REPLAY_*, ARM_*)
bootstrap_arch2.sh   load.py $PROFILES → env BS_* → ros2 run mission_pkg bootstrap_arch2
нода                 BootstrapConfig.from_env(): нет ключа / незнакомый / не тот тип → SystemExit
```

Загрузчик строгий: дубль ключа между профилями, ключ вне схемы (не поле и не
`EXTRA_KEYS`), поле без ключа, значение не того типа или не из `CHOICES` — ошибка с
именами. Дефолт в коде был маскировкой (свип B3s отлетел с ki=0), а не защитой.

| каталог | что внутри |
|---|---|
| `dphold/` | ярус 0 — демпфер `DpHoldM` на IPM-канале: rate-оси, станция, мягкость, перцепция IPM/углы, курс `DpYawHold` |
| `dpvins/` | ярус 1 — ГЕЙНЫ `DpVins` (velocity-каскад, позиционный контур, трим, BRAKE) |
| `vinshold/` | ярус 1 — ГЕЙНЫ И ФЛАГИ `VinsHold` (2D-PID, флаги eagle, Gz-hold, потолки/гейты уверенности осей). Грузится ВСЕГДА рядом с `dpvins/` |
| `vins/` | ярусы 1 и 2 — опора VINS: СЕЛЕКТОР `BS_VINS_STAB` (dpvins \| vinshold, откат — `vins/vinshold.txt`), защёлка трима, хэндовер и зрелость, свежесть, гейт здоровья (три канала), мост VINS→EKF |
| `loiter/` | ярус 2 — штатный LOITER на EKF-от-VINS: гейты миссии, TrackHold/YawBankLimit, GPS-denied/origin/EKF (LV=2 запечён здесь), `BS_EKF_DRAG` и `BS_FCU_PARAMS` для SITL (`rth.txt` = guard + параметры возврата RTL в eeprom, cmd/rth; `smart_rth.txt` — то же + `SRTL_ACCURACY` для возврата по следу, cmd/smart_rth) |
| `wind/` | ярусы 0 и 1 — сквозной ветровой трим `WindTrim` |
| `mission/` | миссия и пилот: `BS_PILOT`/`BS_MISSION`, SF-мастер, кнопка SA, знаки/зона стиков, высота и контур AltHold, бюджеты фаз, геозабор, мягкая посадка, скриптовые миссии; `replay.txt` — реплей пульта |
| `legacy/` | поля ноды ВНЕ активного стека (control_mode/gz-shuttle, DpRollHold/DpPitchHold, старый flow-путь, KF-высота) значениями = прежние дефолты ноды. Кандидат на вычистку из кода вместе с файлом |
| `world/` | ветер Gazebo (`WIND_SPD/DIR_DEG/FACTOR/GUST`) — не ручки ноды (`EXTRA_KEYS`), применяет compose/`capture_scene.sh`; `baseline` = 5 м/с без порывов, `wind2_gust5` (cmd/bl), `wind1_gust8` (history 1…9) |

Эталонный стек (`load.BASELINE_STACK`, = `cmd/bl/bl.sh` WT=1, `check.sh` без
аргументов, тесты `BootstrapConfig.baseline()`): `dphold/baseline dpvins/brake5_stop
vinshold/baseline vins/scale25 loiter/guard wind/trim mission/baseline legacy/baseline
world/wind2_gust5` → 217 ключей (211 полей + 6 внешних).

## Формат

Файл — строки `KEY=VALUE` (без пробелов вокруг `=`), комментарии `#`, пустые строки и
директива **`include <файл>`** (путь относительно каталога самого профиля):

- `include baseline.txt` ставит ВСЕ ключи эталона, строки ниже их ПЕРЕКРЫВАЮТ — кандидат =
  эталон + дельта. Новая ручка (поле датакласса) добавляется в один `baseline.txt`
  подходящего каталога, кандидаты наследуют; забыл — строгая схема упадёт с именем.
- Имя ключа = `BS_` + имя поля в верхнем регистре (`ipm_alt_band_fwd` →
  `BS_IPM_ALT_BAND_FWD`). Прежние псевдонимы (`BS_IPM_ALT_FWD`, `BS_RATE_AWU`,
  `BS_EXCITE_MAX`, `BS_KF_SEG_MIN`, `BS_FLOW_OBS`) переименованы 2026-09-07.
- Типы по аннотации поля: float/int/str/bool (0/1); `Optional[float]` — пустое
  значение = None (`BS_KF_ALT_HOLD=`); строки с `CHOICES` (control_mode, pilot,
  ipm_model, vision_pose_src, alt_src, perc_alt_src, station_frame, vins_vel_src,
  vins_stab) — только из списка.
- Между РАЗНЫМИ перечисленными профилями один ключ — ошибка (каталоги не спорят).
- Внешние ключи (не поля ноды, `config.EXTRA_KEYS`): `BS_EKF_DRAG` (SITL),
  `BS_JOY_DEV` (скрипт пульта), `BS_REPLAY_*` (аргументы реплея), `WIND_*` (мир).
- Профили НЕ для прямого `source` в bash: строка `include` там упадёт — намеренно.

Пометки `# слой:` над блоками — откуда значение пришло ДО 2026-09-07 (история, не
приоритет): **env** (freefly_lv.sh / .env), **нода** (дефолт config.py, теперь
только здесь), **SITL** (`#SITL …` справочно, применяется не через env).

**Не параметры, а аргументы прогона** — в профилях не живут: `BS_REPLAY_SCENARIO`,
`BS_REPLAY_RAW`, `BS_REPLAY_FENCE` (в команде запуска, capture_scene пробрасывает).
Машинное (`VINS_SRC`, `CUDA_ARCH_BIN`, `WORLD`, `LV`) — compose/`docker/sim/.env`
(`docker/sim/env.md`).

## Как применить

```bash
bash cmd/bl/bl.sh                      # cmd/<имя>/<имя>.sh = export PROFILES="…" + freefly_lv
python3 src/control/profiles/load.py dphold/baseline … world/wind2_gust5      # что соберётся
python3 src/control/profiles/load.py --origin --format plain …  # откуда каждый ключ
python3 src/control/profiles/load.py --no-strict dpvins/ki30    # частичный набор, без схемы
bash src/control/profiles/check.sh <RUN>.env [профили…]        # мета прогона == профили?
python3 src/control/profiles/test_load.py                       # контракт + инварианты репо
```

Другой kp = файл-кандидат (`dpvins/<имя>.txt`: include baseline + дельта + гипотеза в
шапке) + копия `cmd/bl` в `cmd/<имя>/` с ним в списке. Откат яруса 1 на VinsHold —
`vins/vinshold` вместо `vins/<…>`. Реплей — `mission/replay` (bl.sh подставляет при
`BS_PILOT=replay`). Мета `<RUN>.env` — полный снимок (`PROFILES=` + все 217 ключей):
`check.sh` печатает расходящиеся ключи (код 1), «ключей профилей нет в мете» и
«BS_ меты вне профилей» должны быть 0 для прогонов с 2026-09-07 (у старых мет —
переименованные ключи и недостающие поля, это нормально).

В коде: `BootstrapConfig.from_env()` (нода), `from_env_file(<RUN>.env)` /
`from_run()` (стенды: env → мета → `PROFILES`), `from_profiles([...])`,
`baseline(**override)` (тесты: эталон + явные переопределения; легаси-путь =
`mission='', stab='', pilot='scripted'`).

## Доказательство 2026-09-07

- Этапы 1–2: стек `cmd/bl` до/после — 134 прежних ключа бит в бит; кандидаты после
  конверсии на `include` разрешаются в свои прежние наборы; полёт
  `lv2_joy_20260907_134836` — расхождений с метой 0.
- Этапы 3–6: конфиг ноды по старому пути (проводка `bootstrap_arch2.sh` → argparse,
  снимок в контейнере) против `BootstrapConfig.from_profiles(BASELINE_STACK)` —
  211 полей, расхождений 0 (bool 0/1 ≡ True/False). Тесты: mission 7/7, control
  31/31, `test_load` 14/14.

## Что сравнивать

Полёты по ярусам под порывами меряет `src/lab/gust_hold_compare.py` (пик за порыв =
жёсткость, остаток за цикл = якорь); гейт здоровья — `vins_sane_replay.py`. Итог эталона
2026-09-05: DpHold пик 2.4–2.8 м, DpVins 6.3–9.5, остаток за цикл DpHold 0.8–1.5 (копится
по ветру), DpVins 0.1–0.4 (возвращается) — память `dphold-vs-dpvins-gusts`.
