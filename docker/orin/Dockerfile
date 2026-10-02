# Официальный образ NVIDIA для Jetson (L4T r36 = JetPack 6.x)
# Содержит: ROS2 Humble (СОБРАН ИЗ ИСХОДНИКОВ в /opt/ros/humble/install) + CUDA 12.2 +
# OpenCV 4.8.1 с CUDA и GStreamer (deb opencv-dev) + PyTorch.
# В базе уже есть cv_bridge, image_transport, vision_msgs, rmw_cyclonedds_cpp (собраны
# против этого OpenCV). ⚠️ НЕ ставить ros-humble-* из apt: потянут второй ROS и системный
# OpenCV 4.5 без CUDA → ABI-краш cv_bridge (та же беда, что в симе). Чего в базе нет —
# собираем из исходников ниже (deps_ws).
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive

# openh264enc (openhd_streamer) — в plugins-bad; python3-serial — отвод пульта CRSF
# (crsf_joy, /dev/ttyTHS1); ceres/glog/boost — VINS.
# Репозиторий packages.ros.org из базы отключаем: ros-humble-* из apt нам запрещены (см.
# выше), а его ключ в базе просрочен (EXPKEYSIG F42ED6FBAB17C654) — apt-get update падал.
RUN grep -l packages.ros.org /etc/apt/sources.list.d/* | xargs -r rm -f && \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    nano \
    v4l-utils \
    libboost-all-dev \
    libceres-dev \
    libgoogle-glog-dev \
    python3-serial \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Сообщения, которых нет в базе: mavros_msgs (лётная нода, ray_tracer) и его
# зависимость geographic_msgs. Только msgs — сам MAVROS крутится на ХОСТЕ борта
# (mavros.service), контейнер говорит с ним по DDS. Версии = apt-пакетам ХОСТА борта
# (ros-humble-mavros-msgs 2.14.0, geographic-msgs 1.0.6): разошлись определения — DDS
# не сматчит типы. Обновили mavros на хосте → поднять теги здесь.
RUN mkdir -p /opt/deps_ws/src && cd /opt/deps_ws/src && \
    git clone --depth 1 -b 1.0.6 https://github.com/ros-geographic-info/geographic_info.git && \
    git clone --depth 1 -b 2.14.0 https://github.com/mavlink/mavros.git && \
    cd /opt/deps_ws && . /opt/ros/humble/install/setup.sh && \
    colcon build --merge-install --packages-select geographic_msgs mavros_msgs \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log src

# Настоящие нейросети на Orin пока не запускаются (только openhd_streamer, см.
# CLAUDE.md). Когда поедут: NN1 — torch+lightglue, NN2 — faiss + DINOv2(torch.hub):
#   RUN pip3 install --no-cache-dir faiss-cpu  # (torch уже в базе dustynv)

WORKDIR /root/vins_ws

RUN echo "source /opt/ros/humble/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/deps_ws/install/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/vins_ws/install/setup.bash ]; then source /root/vins_ws/install/setup.bash; fi" >> /root/.bashrc

ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["tail", "-f", "/dev/null"]
