# ============================================================================
# Контейнер NAV (бывший ai_brain): VINS-Mono + нейросети навигации.
#
# База: nvidia/cuda на Ubuntu 22.04 (jammy) — совпадает с ROS2 Humble.
# CUDA нужна для нейросетей (YOLOv8 / SuperPoint+LightGlue / DINOv2+FAISS).
# VINS-Mono (feature_tracker + vins_estimator) собирается через colcon.
#
# Контекст сборки = корень репозитория (см. docker-compose.yml).
# camera_pkg сюда НЕ входит: в симуляции кадры приходят из Gazebo-топика,
# а не с V4L2-камеры, и CUDA-дебайеризация не нужна.
# ============================================================================
FROM nvidia/cuda:12.2.2-cudnn8-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# --- 1. Локаль + apt-репозиторий ROS2 ---
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --- 2. ROS2 Humble (base, без графики) + MAVROS + сборочный тулчейн ---
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential cmake git \
    libopencv-dev \
    libboost-all-dev \
    libceres-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# --- 3. Геомагнитные датасеты для MAVROS (иначе нода падает на старте) ---
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
       -O /tmp/geo.sh \
    && bash /tmp/geo.sh && rm /tmp/geo.sh

# --- 4. Место под PyTorch + нейросети (раскомментировать при подключении ИИ) ---
# RUN pip3 install --no-cache-dir torch torchvision \
#       --index-url https://download.pytorch.org/whl/cu121 \
#     && pip3 install --no-cache-dir faiss-gpu lightglue

WORKDIR /root/sim_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/sim_ws/install/setup.bash ]; then source /root/sim_ws/install/setup.bash; fi" >> /root/.bashrc

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["tail", "-f", "/dev/null"]
