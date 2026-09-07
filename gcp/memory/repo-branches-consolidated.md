---
name: repo-branches-consolidated
description: С 2026-09-07 в репо 13.17 единственная ветка — main (локально и на origin); все рабочие ветки nn2_* влиты fast-forward и удалены; имена веток в старых записях памяти — исторические метки, коммиты живут в main
metadata:
  type: project
---

2026-09-07 репозиторий `linux100talion/13.17` консолидирован в одну ветку **`main`**:
`nn2_c3_laptop_wind` (вершина 0399d4d, WindTrim возвращён, cmd/bl = baseline) влита в `main`
fast-forward (старый main 2379fae был её прямым предком, 628 коммитов вперёд, 0 назад —
ни merge-коммита, ни force). Затем удалены ВСЕ остальные ветки: на origin 17 штук
(nn2-concept-notes, nn2-dataset-registration, nn2-fusion-notes, nn2_c2_laptop_eagle, nn2_c3,
nn2_c3_control, nn2_c3_control2, nn2_c3_cpu, nn2_c3_laptop, nn2_c3_laptop_vins,
nn2_c3_laptop_wind, nn2_c3_laptop_yaw, nn2_c3_vins_althold и _2…_5) и 5 локальных
(eagle, laptop, laptop_vins, laptop_wind, laptop_yaw). Перед удалением каждая проверена
`merge-base --is-ancestor`; единственная не-предок `nn2-fusion-notes` (5 doc-коммитов
2026-06-18 по nn2_navigation_dream.txt) была перебазированным дубликатом — те же патчи
уже в main под другими хэшами через nn2-dataset-registration, блоб файла бит-в-бит равен.
Локальный `main` привязан к `origin/main`.

**Why:** линия разработки была линейной цепочкой веток (laptop → vins → yaw → eagle → wind),
каждая новая — продолжение предыдущей; держать 17 дублей на GitHub незачем, а «на какой
ветке коммит X» уже зафиксировано в памяти датами и хэшами.

**How to apply:** работать и пушить в `main` (или в новую ветку ОТ main под задачу, влить
и удалить по завершении — не плодить вечные ветки-эпохи). Упоминания «ветка nn2_c3_laptop_yaw»,
«ветка nn2_c2_laptop_eagle» и т.п. в других записях памяти ([[damper-low-alt]],
[[loiter-yaw-dive]], [[sf-master-ladder]], [[lv-loiter-series]], [[vins-solver-fix]],
[[joystick-replay-series]], [[sim-wind-gusts]], [[ekf3-drag-wind]], [[ipm-wz-bias-trap]],
[[control-refactor-split-2026-09]], [[openhd-debug-hud]], [[sa-soft-land]]) — читать как
исторические метки эпохи, коммиты по хэшам ищутся в `git log main`. Форка VINS это не
касается — там по-прежнему ветка `1317_debug` ([[vins-fork-location]]).
