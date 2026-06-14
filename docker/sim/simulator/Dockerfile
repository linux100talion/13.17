# ============================================================================
# Контейнер СИМУЛЯТОРА: Gazebo Harmonic + ArduPilot SITL + мост ros_gz
#
# База: osrf/ros:humble-desktop (x86_64) — полный ROS2 Humble + rviz.
# Внимание: Harmonic — НЕ штатная пара для Humble (штатная — Fortress).
# Ставим Harmonic вручную из репозитория osrfoundation, как требует concept.txt
# (модель использует плагины gz-sim-*).
# ============================================================================
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- 1. Базовые утилиты + apt-репозиторий Gazebo (osrfoundation) ---
RUN apt-get update && apt-get install -y \
    lsb-release wget gnupg curl git \
    build-essential cmake \
    rapidjson-dev \
    libopencv-dev \
    python3-pip \
    && wget https://packages.osrfoundation.org/gazebo.gpg \
       -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# --- 2. Gazebo Harmonic + мост ros_gz для Harmonic + CycloneDDS ---
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# --- 3. ArduPilot SITL (полётный контроллер в софте) ---
# Клонируем исходники, ставим зависимости, собираем arducopter под SITL.
WORKDIR /root
RUN git clone --recurse-submodules --depth 1 -b Copter-4.5 \
        https://github.com/ArduPilot/ardupilot.git
WORKDIR /root/ardupilot
RUN USER=root Tools/environment_install/install-prereqs-ubuntu.sh -y || true
# waf-сборка SITL (--board sitl). Среда из prereqs уже в ~/.bashrc.
RUN . ~/.profile 2>/dev/null; . ~/.bashrc 2>/dev/null; \
    ./waf configure --board sitl && ./waf copter
ENV PATH="/root/ardupilot/build/sitl/bin:/root/ardupilot/Tools/autotest:${PATH}"

# --- 4. Плагин ardupilot_gazebo (мост ArduPilot <-> Gazebo Harmonic) ---
ENV GZ_VERSION=harmonic
WORKDIR /root
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git
RUN cd ardupilot_gazebo && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j$(nproc)
# Регистрируем плагин и модели в путях Gazebo Harmonic.
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
# /root/worlds — смонтированный docker/sim/worlds (миры + дрон-модель)
# /root/worlds/mili_tech — папка с моделями fortress (нужно, чтобы model://<name>
#   резолвились: grass_plane, mili_map, tank1, soldier, oak_tree, ...)
ENV GZ_SIM_RESOURCE_PATH=/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds:/root/worlds:/root/worlds/mili_tech:${GZ_SIM_RESOURCE_PATH}

WORKDIR /root

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["tail", "-f", "/dev/null"]
