FROM ros:iron-ros-base-jammy

WORKDIR /opt/ws

RUN apt-get update && apt-get install curl && rm -rf /var/lib/apt/lists/*

RUN mkdir src && \
    cd src && \
    git clone --depth=1 https://github.com/davidliyutong/optitrack_bridge && \
    cd optitrack_bridge && \
    git submodule update --init --depth=1 third_party/yaml-cpp

RUN cd src/optitrack_bridge && bash ./scripts/install_sdk.bash && ln -s manifests/package.xml/ros2.xml package.xml

RUN colcon build --symlink-install


