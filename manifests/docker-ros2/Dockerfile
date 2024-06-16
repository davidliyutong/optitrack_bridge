FROM ros:iron-ros-base-jammy

WORKDIR /opt/ws

RUN apt-get update && apt-get install curl && rm -rf /var/lib/apt/lists/*

RUN mkdir src && \
    cd src && \
    git clone --depth=1 https://github.com/davidliyutong/optitrack_bridge && \
    cd optitrack_bridge && \
    git submodule update --init --recursive --depth=1  \

RUN bash ./scripts/install_sdk.bash

RUN colcon build --symlink-install


