# Официальный образ NVIDIA для Jetson (L4T r36 = JetPack 6.x)
# Содержит: ROS2 Humble + CUDA 12.2 + OpenCV с CUDA + PyTorch
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    nano \
    v4l-utils \
    libboost-all-dev \
    libceres-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/vins_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/vins_ws/install/setup.bash ]; then source /root/vins_ws/install/setup.bash; fi" >> /root/.bashrc

ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["tail", "-f", "/dev/null"]
