# ============================================================================
# Обёртка над симуляционным стеком: ArduPilot SITL + Gazebo Harmonic + VINS/nav.
#
# Типичная итерация разработки (НЕ удаляет контейнеры — быстро):
#   make restart-all      # stop → start; скрипты запускаются автоматически
#   make wait             # ждать пока nav_up.sh завершится («nav: готово»)
#   make status           # что запустилось (✓/✗), последние ошибки из логов
#   make logs             # непрерывный хвост output/*.log
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
# GPU-less прогон (машина без NVIDIA, ветка nn2_c3_cpu) — добавить CPU=1 к
# ЛЮБОМУ таргету: подключает docker-compose.cpu.yml (runc вместо nvidia-runtime,
# софтовый GL для Gazebo, drop-in CPU-камера camera_node_cpu). Флаг держать во
# всех командах сессии, иначе compose адресует другой набор файлов:
#   make CPU=1 build && make CPU=1 up && make CPU=1 logs
#   make CPU=1 restart-all && make CPU=1 wait
#
# Запускать из каталога docker/sim/.
# ============================================================================

# CPU=1 — наложить CPU-оверрайд (GPU-less режим) на все docker compose команды.
CPU ?=
ifeq ($(CPU),1)
DC := docker compose -f docker-compose.yml -f docker-compose.cpu.yml
else
DC := docker compose
endif

SIM     := p1317_simulator
NAV     := p1317_nav
DISPLAY ?= :0
export DISPLAY

.DEFAULT_GOAL := help
.PHONY: help host-setup build up sim nav run stop restart-all fresh-start down ps logs status wait nav-rebuild arm takeoff hover land disarm fly vins-watch capture-frames capture-scene clean

# Дефолтная последовательность команд для `make capture-scene` (без разрешения →
# restart-all). Переопределяется: make capture-scene CSARGS="640x480 arm takeoff 5 hover 2 land"
CSARGS ?= arm takeoff 4 hover 20 land

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

ps:              ## статус контейнеров (docker compose ps)
	$(DC) ps

status:          ## что запустилось, что упало (процессы + последние ошибки)
	@echo "── simulator ──────────────────────────────────"
	@docker exec $(SIM) bash -c '\
	  ok(){ pgrep -f "$$1" >/dev/null && echo "  ✓ $$2" || echo "  ✗ $$2 — НЕ ЗАПУЩЕН"; }; \
	  ok "gz sim"          "gz sim (Gazebo)"; \
	  ok "arducopter"      "SITL (arducopter)"; \
	  ok "ros_gz_bridge"   "ros_gz_bridge"; \
	  echo; \
	  echo "  -- последние ошибки sitl.log --"; \
	  grep -i "error\|crash\|abort\|fail" /root/output/sitl.log 2>/dev/null | tail -3 || true'
	@echo "── nav ─────────────────────────────────────────"
	@docker exec $(NAV) bash -c '\
	  ok(){ pgrep -f "$$1" >/dev/null && echo "  ✓ $$2" || echo "  ✗ $$2 — НЕ ЗАПУЩЕН"; }; \
	  ok "bayerizer\.py"                   "bayerizer"; \
	  ok "camera_pkg/lib/camera_pkg"       "camera_node"; \
	  ok "feature_tracker/feature_tracker" "feature_tracker"; \
	  ok "vins_estimator/vins_estimator"   "vins_estimator"; \
	  ok "mavros_node"                     "MAVROS"; \
	  echo; \
	  echo "  -- последние ошибки sim_nav.log --"; \
	  grep -i "\[ERROR\]\|died\|abort\|NumPy\|disorder" /root/sim_ws/output/sim_nav.log 2>/dev/null | tail -5 || true; \
	  echo "  -- mavros --"; \
	  grep -i "error\|FCU:\|connected\|timeout" /root/sim_ws/output/mavros.log 2>/dev/null | tail -3 || true'

wait:            ## ждать пока nav_up.sh напишет «nav: готово» (до 5 мин)
	@echo "Жду завершения nav_up.sh..."
	@timeout 300 bash -c '\
	  until docker logs p1317_nav 2>&1 | grep -q "nav: готово"; do sleep 3; done' \
	  && echo "nav: готово" || echo "TIMEOUT (5 мин)"

logs:            ## хвост логов всех нод (output/*.log)
	tail -n 40 -f output/*.log

nav-rebuild:     ## принудительная пересборка colcon workspace в nav
	docker exec $(NAV) bash -c 'cd /root/sim_ws && rm -rf build/* install/* log'
	$(MAKE) nav

arm:             ## GUIDED + арм (без взлёта)
	docker exec $(NAV) bash /lab/arm.sh

takeoff:         ## взлёт на ALT м (ALT=3 по умолчанию; нужен предварительный arm)
	docker exec $(NAV) bash /lab/takeoff.sh $(ALT)

hover:           ## висение SEC секунд sim-времени (SEC=10 по умолчанию)
	docker exec $(NAV) bash /lab/hover.sh $(SEC)

land:            ## посадка (режим LAND)
	docker exec $(NAV) bash /lab/land.sh

disarm:          ## дизарм
	docker exec $(NAV) bash /lab/disarm.sh

fly:             ## запустить облёт квадрата для инициализации VINS (Ctrl+C для стопа)
	docker exec -it $(NAV) python3 /lab/fly_square.py $(FLYARGS)

vins-watch:      ## мониторинг VINS: Initialization/NON_LINEAR/ошибки + odometry Hz
	docker exec -i $(NAV) bash /lab/vins_watch.sh

capture-frames:  ## снять N кадров из Gazebo мира → worlds/preview/*.png  (N=5 default)
	bash scripts/capture_frames.sh $(N)

capture-scene:   ## секвенсор: рестарт→[WxH]→команды→запись→кадры→Google Drive (CSARGS=...)
	bash ../../src/lab/capture_scene.sh $(CSARGS)

clean: down      ## down + удалить образы и тома стека
	$(DC) down --rmi local --volumes
