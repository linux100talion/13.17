# Memory Index

- [bootstrap-excite-tuning](bootstrap-excite-tuning.md) — почему дрон улетал в дом (размах a·τ²), формула тюнинга, цель «пятачок 3-5м / 60с»
- [imu-sim-freq-sim](imu-sim-freq-sim.md) — IMU sim-частота ≈50Гц raw/30Гц filt, потолок SITL, доки про ≥80 устарели
- [yaw-hold-tuning](yaw-hold-tuning.md) — YAW-hold: победитель kp6/ki0/sm5; ki ВРЕДЕН (bias yaw_flow); сглаживание = временное окно (fps-зависимо)
- [control-refactor-arch](control-refactor-arch.md) — рефакторинг alt_hold_bootstrap→src/control (hexagonal/DDD), ветка nn2_c3_control, 3 роли, пульт-как-стратегия, срез 1 = gz-hold+shuttle
- [always-upload-video](always-upload-video.md) — прогоны в симе ВСЕГДА с GDRIVE_UP=1 MP4=1 (видео на Drive; юзер смотрит с телефона)
- [user-works-from-field](user-works-from-field.md) — телефон + Starlink из ямы, связь рвётся: коротко, отвязанно, цепочку вести самому
- [no-edit-scripts-midrun](no-edit-scripts-midrun.md) — не править src/lab/*.sh во время прогона: bash сдвигает офсет, миссия уходит на второй круг
- [setpoint-integrator-injection](setpoint-integrator-injection.md) — впрыск команд через интегратор уставки: механизм ОК, врёт канал зрения (гейн 0.35..1.8)
- [run-config-no-sed](run-config-no-sed.md) — конфиг прогона писать heredoc'ом целиком: sed по `N=1` задел OSIGN и CMD_GAIN
