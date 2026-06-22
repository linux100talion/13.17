# Memory index

- [GCE GPU plan](gce-gpu-plan.md) — build-box dev-workspace-1317: Docker установлен, оба sim-образа собраны (sim-simulator 7.6 GB + sim-nav 27.2 GB); квота GPUS_ALL_REGIONS поднята; GPU-апгрейд через 08_add_gpu.sh запускать из Google Cloud Shell
- [User setup](user-setup.md) — работает с телефона, ноута нет; для внешнего gcloud использовать Google Cloud Shell
- [VINS blocker](vins-blocker.md) — устарело, см. vins-init
- [VINS init debug](vins-init.md) — check2+extrinsic+td исправлены; БЛОКЕР: scale=0.02 (features далеко) + gravity direction систематически wrong ([9.5,0,-2.5] вместо [0,9.5,2.5]) → failure detection каждые ~30с
- [Sim workflow](sim-workflow.md) — make restart-all/fresh-start/status/wait; что теряется при fresh-start; патчи nav_up.sh; команды арминга
