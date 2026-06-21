# Memory index

- [GCE GPU plan](gce-gpu-plan.md) — build-box dev-workspace-1317: Docker установлен, оба sim-образа собраны (sim-simulator 7.6 GB + sim-nav 27.2 GB); квота GPUS_ALL_REGIONS поднята; GPU-апгрейд через 08_add_gpu.sh запускать из Google Cloud Shell
- [User setup](user-setup.md) — работает с телефона, ноута нет; для внешнего gcloud использовать Google Cloud Shell
- [VINS blocker](vins-blocker.md) — стек запущен, дрон взлетает, но VINS не удерживает NON_LINEAR: Initialization finish! каждые 10с, /odometry не публикуется; возможная причина — IMU патч dt=1e-6 или шум в sim.yaml
- [Sim workflow](sim-workflow.md) — make restart-all/fresh-start/status/wait; что теряется при fresh-start; патчи nav_up.sh; команды арминга
