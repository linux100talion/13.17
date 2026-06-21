# ============================================================================
# Обёртка над симуляционным стеком: ArduPilot SITL + Gazebo Harmonic + VINS/nav.
# Порядок запуска:  make host-setup -> build -> up -> sim -> nav  (или make run).
#
# Скрипты исполняются ВНУТРИ контейнеров через `docker exec -i ... bash -s`
# (передаются по stdin — монтировать scripts/ не нужно).
# Запускать из каталога docker/sim/.
# ============================================================================

DC      := docker compose
SIM     := p1317_simulator
NAV     := p1317_nav
DISPLAY ?= :0
export DISPLAY

.DEFAULT_GOAL := help
.PHONY: help host-setup build up sim nav run stop down restart ps logs nav-rebuild capture-frames clean

help:            ## показать список целей
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-12s\033[0m %s\n",$$1,$$2}'

host-setup:      ## хост: xhost + v4l2loopback (/dev/rawbayer) — нужен sudo
	bash scripts/host_setup.sh

build:           ## собрать образы (долго: SITL+Gazebo+OpenCV из исходников)
	$(DC) build

up:              ## поднять контейнеры (-d)
	$(DC) up -d

sim:             ## в simulator: Gazebo + SITL + ros_gz_bridge (фон, логи в output/)
	docker exec -i $(SIM) bash -s < scripts/sim_up.sh

nav:             ## в nav: colcon build (если надо) + sim_nav.launch + MAVROS
	docker exec -i $(NAV) bash -s < scripts/nav_up.sh

run: up sim nav  ## всё разом: up -> sim -> nav

stop:            ## остановить процессы в контейнерах (сами контейнеры живут)
	-docker exec $(SIM) pkill -f 'gz sim|sim_vehicle|ros_gz_bridge'
	-docker exec $(NAV) pkill -f 'sim_nav.launch|mavros_node|camera_node|feature_tracker|vins_estimator|bayerizer'

restart: stop sim nav  ## перезапустить ноды без пересборки образов

down:            ## погасить и удалить контейнеры
	$(DC) down

ps:              ## статус контейнеров
	$(DC) ps

logs:            ## хвост логов запущенных нод (output/*.log)
	tail -n 40 -f output/*.log

nav-rebuild:     ## принудительная пересборка workspace в nav (colcon)
	docker exec $(NAV) bash -c 'cd /root/sim_ws && rm -rf build install log'
	$(MAKE) nav

capture-frames:  ## снять N кадров из Gazebo мира → worlds/preview/*.png  (N=5 default, N=10 пример)
	bash scripts/capture_frames.sh $(N)

clean: down      ## down + удалить образы и тома стека
	$(DC) down --rmi local --volumes
