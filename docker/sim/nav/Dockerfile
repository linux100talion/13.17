# ============================================================================
# Контейнер NAV (бывший ai_brain): VINS-Mono + камера-нода + нейросети.
#
# Вариант А: OpenCV собирается ИЗ ИСХОДНИКОВ с CUDA, чтобы камера-нода
# (camera_pkg, cv::cuda::demosaicing) работала в симуляции "как есть", читая
# кадры из виртуального устройства /dev/rawbayer (v4l2loopback).
#
# База devel (а не runtime) — нужен nvcc для сборки CUDA-модулей OpenCV.
# Контекст сборки = корень репозитория (см. docker-compose.yml).
#
# ВАЖНО про ABI: apt'шный ros-humble-cv-bridge тянет системный OpenCV (4.5.4,
# без CUDA). Линковать его рядом с нашим 4.10+CUDA нельзя — будет краш.
# Поэтому cv_bridge НЕ ставим из apt, а собираем из исходников против /usr/local.
# ============================================================================
FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Список compute capability под GPU ноута. По умолчанию покрывает Turing→Ada
# (RTX 20/30/40). Для ускорения сборки задай свою: --build-arg CUDA_ARCH_BIN=8.9
ARG CUDA_ARCH_BIN="7.5;8.0;8.6;8.9"
ARG OPENCV_VERSION="4.10.0"

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

# --- 2. ROS2 Humble (base) + MAVROS + тулчейн + зависимости сборки OpenCV ---
# Намеренно НЕТ ros-humble-cv-bridge: его соберём из исходников против /usr/local.
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-vision-msgs \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-colcon-common-extensions \
    python3-pip python3-dev python3-numpy \
    build-essential cmake git pkg-config wget unzip \
    v4l-utils \
    libboost-all-dev libceres-dev libeigen3-dev \
    libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-tools \
    libjpeg-dev libpng-dev libtiff-dev \
    libtbb-dev libatlas-base-dev gfortran \
    && rm -rf /var/lib/apt/lists/*

# --- 3. Сборка OpenCV + opencv_contrib с CUDA (модули cudaimgproc/cudaarithm) ---
# cudaimgproc (demosaicing) и cudaarithm живут в opencv_contrib.
WORKDIR /opt/opencv_build
RUN wget -q -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && wget -q -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
    && unzip -q opencv.zip && unzip -q opencv_contrib.zip \
    && rm opencv.zip opencv_contrib.zip
RUN cd opencv-${OPENCV_VERSION} && mkdir build && cd build \
    && cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_build/opencv_contrib-${OPENCV_VERSION}/modules \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D OPENCV_DNN_CUDA=ON \
        -D CUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
        -D CUDA_FAST_MATH=ON \
        -D ENABLE_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_V4L=ON \
        -D BUILD_opencv_python3=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_opencv_apps=OFF \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && cd / && rm -rf /opt/opencv_build

# --- 4. cv_bridge из исходников против нашего OpenCV (overlay /opt/overlay) ---
RUN mkdir -p /opt/overlay/src \
    && cd /opt/overlay/src \
    && git clone --depth 1 -b humble https://github.com/ros-perception/vision_opencv.git
RUN cd /opt/overlay \
    && . /opt/ros/humble/setup.sh \
    && colcon build --packages-select cv_bridge image_geometry \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# --- 5. Геомагнитные датасеты для MAVROS (иначе нода падает на старте) ---
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
       -O /tmp/geo.sh \
    && bash /tmp/geo.sh && rm /tmp/geo.sh

# --- 6. PyTorch + LightGlue (SuperPoint) для Нейросети №1 (nn1_anchor) ---
# PyTorch несёт свою CUDA-рантайм-сборку — отдельно ставить CUDA не нужно.
# ВАЖНО: lightglue тянет в зависимостях opencv-python (pypi, без CUDA/GStreamer),
# который ШАДОВИТ наш OpenCV из исходников и ломает cv2 в python-нодах
# (openhd_streamer: VideoWriter CAP_GSTREAMER). Поэтому ставим lightglue
# --no-deps, а нужный рантайм-деп (kornia, без opencv) — отдельно.
RUN pip3 install --no-cache-dir torch torchvision \
        --index-url https://download.pytorch.org/whl/cu121 \
    && pip3 install --no-cache-dir kornia \
    && pip3 install --no-cache-dir --no-deps \
        git+https://github.com/cvg/LightGlue.git
# Ретривал-префильтр (Инкремент 1.x / Нейросеть №2): faiss-gpu + DINOv2 — позже.
# RUN pip3 install --no-cache-dir faiss-gpu

WORKDIR /root/sim_ws

# Порядок source важен: сначала ROS, затем overlay (cv_bridge против CUDA-OpenCV),
# затем workspace. Так colcon в /root/sim_ws линкует именно наш OpenCV.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/overlay/install/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/sim_ws/install/setup.bash ]; then source /root/sim_ws/install/setup.bash; fi" >> /root/.bashrc

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["tail", "-f", "/dev/null"]
