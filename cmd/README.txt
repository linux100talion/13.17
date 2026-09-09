cmd/ — команды запуска полётов и кампаний
=========================================

Договорённость (2026-09-05): каждый скрипт запуска живёт в СВОЕЙ папке с тем же
именем, рядом с ним README.txt:

  cmd/<имя>/<имя>.sh    — сам скрипт (профили → env → src/lab/freefly_lv.sh …)
  cmd/<имя>/README.txt  — зачем создан, что меняет против эталона и для чего,
                          как летать, чем судить, результат (дописывается после полёта)

Скрипты не зависят от текущего каталога (корень репы вычисляется от своего
расположения) и ДЕРЖАТ ТОЛЬКО СПИСОК профилей — ни eval, ни export ручек:
    export PROFILES="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25
                     loiter/guard wind/trim mission/baseline legacy/baseline world/wind2_gust5"
    exec bash src/lab/freefly_lv.sh "$@"
(с 2026-09-07). Список собирает freefly_lv.sh на хосте (мета, eeprom, ветер compose) и
bootstrap_arch2.sh в контейнере (env ноды) — один load.py, строго по схеме
BootstrapConfig: дубль/незнакомый/отсутствующий ключ = ошибка. Ручку через env
переопределить НЕЛЬЗЯ по построению (BS_*/WIND_* из env и .env не читаются) —
только файл-кандидат (include baseline + дельта) + копия cmd/bl → cmd/<имя>/.
Каталоги: mission/ (пилот, миссия, посадка, бюджеты), legacy/ (поля вне стека),
world/ (ветер). Аргументы прогона — не параметры: BS_PILOT=replay + BS_REPLAY_SCENARIO
(сценарий) едут env'ом. См. src/control/profiles/README.md, docker/sim/env.md.

Раскладка (с 2026-09-06):

  cmd/bl/bl.sh                 — ТЕКУЩИЙ BASELINE: то, чем летаем по умолчанию (сейчас —
                                 бывший cmd/11: cmd/10 + WindTrim, плечо WT=0|1). Новый
                                 кандидат делается копией bl → cmd/<имя>/, доказанный
                                 кандидат становится новым bl.
  cmd/rth/, cmd/smart_rth/     — ВОЗВРАТ ДОМОЙ (кампания 2026-09-07): тот же шаг Rth
                                 плана freefly, режим выбирает топик — RTL (прямая на
                                 home, `make rth`) и SMART_RTL (по крошкам пройденного
                                 пути, `make smart-rth`). Ручки стека = cmd/bl бит в
                                 бит, отличие — профиль loiter/rth|smart_rth (параметры
                                 FCU в eeprom). Мерка — src/lab/rth_check.py.
  cmd/rth_track/               — ВОЗВРАТ ПО СВОЕМУ ТРЕКУ (2026-09-09): GUIDED + поток
                                 уставок, курс по треку, дома сразу мягкая посадка —
                                 вместо крошек SMART_RTL полётника (их 300-500 ≈ 1 км).
                                 Профиль loiter/rth_track (WPNAV_* под наш масштаб);
                                 механика — src/mission/rth.md.
  cmd/yaw_vins/                — КУРС EKF ОТ VINS (2026-09-09): то же, что cmd/rth_track,
                                 плюс EK3_SRC1_YAW=6 после латча возврата (компас уходит
                                 из контура). Профиль mission/yaw_vins; механика и риски —
                                 src/nav/frames.md.
  cmd/history/<кампания>/<n>/  — архив отлетавших скриптов как есть (README с результатом);
                                 ссылки «cmd/<n>» в доках/памяти/профилях = сюда
                                 (wind/1…11 — кампания ветра/станции 2026-09-05…06).

КАРТА ПУЛЬТА (какой канал/орган TX12 за что отвечает: CH1..CH8 и кнопки в /joy, ярусы
по SC/SF, кнопка SA, жесты арм/дизарм, эквиваленты с хоста make sa-land/rth/smart-rth,
чем мерить и где ручки) — docker/sim/rx.md.
