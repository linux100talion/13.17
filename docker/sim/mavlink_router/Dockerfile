FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git meson ninja-build pkg-config \
    gcc g++ \
    libglib2.0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/mavlink-router/mavlink-router.git /opt/mavlink-router
WORKDIR /opt/mavlink-router
RUN git submodule update --init --recursive \
    && meson setup build . -Dsystemdsystemunitdir=/lib/systemd/system \
    && ninja -C build \
    && ninja -C build install

CMD ["mavlink-routerd", "-c", "/etc/mavlink-router/main.conf"]
