# OptiTrack Bridge

This is a tool to bridge optitrack NatNetSDK to a gRPC server / a ROS2 topic publisher.

## Build Instructions

### Normal Build

Clone the repository and run `git submodule update --init --recursive` to update submodules.

First run the `scripts/install_sdk.[bash|ps1]` to populate `third_party/NatNetSDK` with the SDK source code.

```shell
bash scripts/install_sdk.bash
```

or

```shell
powershell scripts/install_sdk.ps1
```

> The `envfile|envfile.ps1` script sets the version for grpc and NatNetSDK. The default version is `v1.64.0` for grpc and `4.1.1` for NatNetSDK.

Next, build gRPC library with the `scripts/install_grpc.[bash|ps1]`


```shell
bash scripts/install_grpc.bash
```

> You may need to install the following dependencies 
> - Linux: 
>   ```
>   sudo apt install -y build-essential autoconf libtool pkg-config cmake 
>   ```
> - MacOS: 
>   ```
>   brew install autoconf automake libtool pkg-config cmake
>   ```

or

```shell
powershell scripts/install_grpc.ps1
```

> You will need `Visual Studio 2019` or later, with **CORRECT** Windows SDK installed.
> 
> `nasm` is need and can be installed with [Chocolatey](https://chocolatey.org/install): `choco install nasm`

This may take a while depending on your internet connection and hardware.

After building the dependencies, you can build the project with the following commands:

```shell
mkdir build
cd build
cmake ..
cmake --build . --target optitrack_bridge
cmake --build . --target copy_dll
```

If the build is successful, you can copy the `build/bin` to somewhere convenient. Then run the `optitrack_bridge` executable.


```powershell
optitrack_bridge --config <path_to_config_file> --port <port_number>
```

> The `--config` flag is optional. If not provided, the program will look for `config.yaml` in the current directory, `$HOME/.config/rfmocap/config.yaml` or `/etc/rfmocap/config.yaml`.
> The `--port` flag is optional. If not provided, the program will use the default port `50051`.

Here is an example of a `config.yaml` file:

```yaml
motive:
  server_address: "<your_motive_address>"
```

### ROS1 build

This project can be built with ROS1. First clone the project in a ROS1 workspace:

```shell
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/davidliyutong/optitrack_bridge
```

Then update the submodules and install NatNetSDK with the following commands:

```shell
# catkin_ws/src
cd optitrack_bridge && git submodule update --init --recursive --depth=1
bash ./scripts/install_sdk.bash ## download the sdk pre-built binaries
```

Now, choose the right `package.xml`. Since we are using ROS1, we need to use the `ros1.xml` in the `manifests/package.xml` directory. 

```shell
ln -s manifests/package.xml/ros1.xml package.xml
```

Then build the project with the following commands:

```shell
cd catkin_ws/
# catkin_ws/
catkin_make
```

> There is no need to install grpc since we are not launching the gRPC server.
> The ros2 build instruction is only tested on Ubuntu 22.04 with ROS2 Iron.


### ROS2 build

The project can also be built with ROS2. First clone the project in a ROS2 workspace:

```shell
mkdir -p ros_ws/src && cd ros_ws/src
git clone https://github.com/davidliyutong/optitrack_bridge
```

Then update the submodules and install NatNetSDK with the following commands:

```shell
# ros_ws/src
cd optitrack_bridge && git submodule update --init --recursive --depth=1
bash ./scripts/install_sdk.bash ## download the sdk pre-built binaries
```

Now, choose the right `package.xml`. Since we are using ROS2, we need to use the `ros2.xml` in the `manifests/package.xml` directory. 

```shell
ln -s manifests/package.xml/ros2.xml package.xml
```

Then build the project with the following commands:

```shell
cd ros_ws/
# ros_ws/
colcon build --symlink-install # --symlink-install is used to install libNatNet.so
```

> There is no need to install grpc since we are not launching the gRPC server.
> The ros2 build instruction is only tested on Ubuntu 22.04 with ROS2 Iron.


#### Docker Build

You can also build the project with Docker. First, build the docker image with the following command:

```shell
docker build -t optitrack_bridge:ros2-latest -f manifests/docker-ros2/Dockerfile .
```

### Generate Python Client

You can generate a Python client with the following commands:

```shell
make task.pb.python_client
```

The client will be generated in `client/python`

> You need to install `grpc-tools` via `pip install grpc-tools` to perform this operation

## Using the Optitrack Bridge

### Normal Usage

The Optitrack Bridge is a gRPC server that streams Optitrack data to clients. The protocol buffer definition can be found in [lib/tracker_packet/manifests/tracker_packet.proto](./lib/tracker_packet/manifests/tracker_packet.proto).

You can build the example client in the `example` directory with the following commands:

```shell
cd build
cmake --build . --target example_grcp_client
```

Then run the `example_grpc_client` executable.

```shell
example_grpc_client
```

> The example will Dial localhost:50051, make sure the client and the server is on the same host.

### Working with ROS1

After build, create the yaml configuration file:

```shell
cd catkin_ws
echo 'motive:
  server_address: "<your_address>"'> config.yaml
```

Open another ROS terminal and start `roscore`. Then run the following commands:

```shell
# catkin_ws/
source devel/setup.bash
rosrun optitrack_bridge node _config_path:=$(pwd)/config.yaml
```

#### Docker Run

Run the docker container with the following command:

```shell
dockers run -it --rm --net=host optitrack_bridge:ros1-latest
```

### Working with ROS2

After build, create the yaml configuration file:

```shell
cd ros_ws
echo 'motive:
  server_address: "<your_address>"'> config.yaml
```

Then run the following commands:

```shell
source install/setup.bash
ros2 run optitrack_bridge optitrack_ros2_publisher --ros-args -p config_path:=$(pwd)/config.yaml
```

#### Docker Run

Run the docker container with the following command:

```shell
dockers run -it --rm --net=host optitrack_bridge:ros2-latest
```

## TODO

- [ ] Test on ROS1
- [ ] Test on ROS2
- [ ] Test on Linux