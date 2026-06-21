# ============================================================================
# Обёртка над симуляционным стеком: ArduPilot SITL + Gazebo Harmonic + VINS/nav.
#
# Типичная итерация разработки (НЕ удаляет контейнеры — быстро):
#   make restart-all      # stop → start; скрипты запускаются автоматически
#   make logs             # смотреть что поднялось
#
# Первый запуск / пересборка образов:
#   make host-setup       # хост: v4l2loopback + xhost
#   make build            # собрать образы (долго)
#   make up               # создать и запустить контейнеры (скрипты стартуют сами)
#
# Полный сброс (пересоздать контейнеры, потерять ephemeral state):
#   make fresh-start
#
# Ручной перезапуск только нод (не трогая контейнеры):
#   make sim              # перезапустить Gazebo+SITL+bridge внутри simulator
#   make nav              # перезапустить VINS+MAVROS внутри nav
#
# Запускать из каталога docker/sim/.
# ============================================================================

DC      := docker compose
SIM     := p1317_simulator
NAV     := p1317_nav
DISPLAY ?= :0
export DISPLAY

.DEFAULT_GOAL := help
.PHONY: help host-setup build up sim nav run stop restart-all fresh-start down ps logs nav-rebuild capture-frames clean

help:            ## показать список целей
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-12s\033[0m %s\n",$$1,$$2}'

host-setup:      ## хост: xhost + v4l2loopback (/dev/rawbayer) — нужен sudo
	bash scripts/host_setup.sh

build:           ## собрать образы (долго: SITL+Gazebo+OpenCV из исходников)
	$(DC) build

up:              ## создать и запустить контейнеры (скрипты стартуют автоматически)
	$(DC) up -d

sim:             ## (ручной) перезапустить ноды в simulator (Gazebo+SITL+bridge)
	docker exec -i $(SIM) bash -s < scripts/sim_up.sh

nav:             ## (ручной) перезапустить ноды в nav (VINS+MAVROS)
	docker exec -i $(NAV) bash -s < scripts/nav_up.sh

run: up          ## псевдоним для up (скрипты стартуют сами при подъёме контейнеров)

restart-all:     ## быстрый перезапуск: stop → start (контейнеры живут, ephemeral state сохраняется)
	$(DC) stop
	$(DC) start

fresh-start:     ## полный сброс: down → up (пересоздать контейнеры, потерять ephemeral state)
	$(DC) down
	$(DC) up -d

down:            ## погасить и удалить контейнеры
	$(DC) down

ps:              ## статус контейнеров
	$(DC) ps

logs:            ## хвост логов всех нод (output/*.log)
	tail -n 40 -f output/*.log

nav-rebuild:     ## принудительная пересборка colcon workspace в nav
	docker exec $(NAV) bash -c 'cd /root/sim_ws && rm -rf build install log'
	$(MAKE) nav

capture-frames:  ## снять N кадров из Gazebo мира → worlds/preview/*.png  (N=5 default)
	bash scripts/capture_frames.sh $(N)

clean: down      ## down + удалить образы и тома стека
	$(DC) down --rmi local --volumes
