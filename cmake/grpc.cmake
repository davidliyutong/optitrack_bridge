set(GRPC_INSTALL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/grpc/install)
list(APPEND CMAKE_PREFIX_PATH ${GRPC_INSTALL_PATH})

# Find gRPC installation
find_package(absl CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_program(_PROTOBUF_PROTOC protoc)
find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)