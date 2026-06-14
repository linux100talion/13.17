# Берем официальный и абсолютно чистый ROS 2 Humble для ARM64
FROM arm64v8/ros:humble-ros-base-jammy

# Отключаем интерактивные диалоги при установке пакетов
ENV DEBIAN_FRONTEND=noninteractive

# Устанавливаем системные утилиты и правильный cv_bridge (OpenCV 4.5.4)
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-colcon-common-extensions \
    build-essential \
    git \
    nano \
    ros-humble-rmw-cyclonedds-cpp \
    v4l-utils \
    libboost-all-dev \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# Задаем рабочую директорию внутри контейнера
WORKDIR /root/vins_ws

# Прописываем автозагрузку ROS 2 при каждом входе в контейнер
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/vins_ws/install/setup.bash ]; then source /root/vins_ws/install/setup.bash; fi" >> /root/.bashrc

# Жестко фиксируем настройки сети CycloneDDS для всего контейнера
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Оставляем контейнер висеть в фоне, чтобы мы могли в него заходить
CMD ["tail", "-f", "/dev/null"]