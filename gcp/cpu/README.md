# gcp/cpu/ — питание CPU-only бокса

Скрипты управления отдельным **CPU-инстансом** `dev-workspace-1317-cpu`
(без GPU), поднятым под GPU-less прогон `gazebo → SITL → VINS` (ветка
`nn2_c3_cpu`), пока T4 в дефиците.

| Параметр  | Значение                          |
|-----------|-----------------------------------|
| Instance  | `dev-workspace-1317-cpu`          |
| Zone      | `europe-west4-a` (env-override `ZONE=…`) |
| Type      | `c2d-standard-16` (16 vCPU AMD Milan) |
| GPU       | нет                               |

Это **не** GPU-инстанс `dev-workspace-1317` — отдельное имя, отдельный диск.
Хелперы верхнего уровня (`02_power_manager`, `04_…`) и каталоги `spot/`,
`on_demand/` прибиты к имени `dev-workspace-1317` и этот бокс **не трогают**.

## Скрипты

- `start.sh` — запустить (тонкий `gcloud instances start`). CPU-ёмкость
  дефицитом обычно не страдает.
- `stop.sh` — остановить (диск остаётся в зоне, платишь только за storage).
- `ssh.sh` — SSH; без аргументов заходит сразу в `~/13.17`, с аргументами
  (`./ssh.sh --command='docker ps'`) пробрасывает их в `gcloud compute ssh`.

## Создание / удаление

Создание — через `../01_create_workspace.sh` с тогглами (в репе скрипта нет
отдельной обёртки, бокс одноразово создан так):

```bash
GPU=0 MACHINE_TYPE=c2d-standard-16 INSTANCE_NAME=dev-workspace-1317-cpu \
    ./01_create_workspace.sh 120
```

💰 CPU-инстанс тарифицируется, пока `RUNNING` (c2d-standard-16 ≈ $0.75/час
on-demand). Гаси `./stop.sh`, когда не нужен.
