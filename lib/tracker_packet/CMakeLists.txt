get_filename_component(tp_proto "${CMAKE_CURRENT_SOURCE_DIR}/manifests/tracker_packet.proto" ABSOLUTE)
get_filename_component(tp_proto_path "${tp_proto}" PATH)
set(tp_proto_srcs "${CMAKE_CURRENT_SOURCE_DIR}/protos/tracker_packet.pb.cc")
set(tp_proto_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/protos/tracker_packet.pb.h")
set(tp_grpc_srcs "${CMAKE_CURRENT_SOURCE_DIR}/protos/tracker_packet.grpc.pb.cc")
set(tp_grpc_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/protos/tracker_packet.grpc.pb.h")
add_custom_command(
    OUTPUT "${tp_proto_srcs}" "${tp_proto_hdrs}" "${tp_grpc_srcs}" "${tp_grpc_hdrs}"
    COMMAND ${_PROTOBUF_PROTOC}
    ARGS --grpc_out "${CMAKE_CURRENT_SOURCE_DIR}/protos"
    --cpp_out "${CMAKE_CURRENT_SOURCE_DIR}/protos"
    -I "${tp_proto_path}"
    --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
    "${tp_proto}"
    DEPENDS "${tp_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/protos")
add_library(tracker_packet_grpc_proto
    ${tp_grpc_srcs}
    ${tp_grpc_hdrs}
    ${tp_proto_srcs}
    ${tp_proto_hdrs})
target_link_libraries(tracker_packet_grpc_proto
    gRPC::grpc++ gRPC::grpc++_reflection protobuf::libprotobuf)


# Add the implementation

set(TRACKER_PACKET_INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB SERVER_SOURCES "src/server_*.cxx")
add_library(tracker_packet_server ${SERVER_SOURCES})
target_include_directories(tracker_packet_server PUBLIC ${TRACKER_PACKET_INCLUDE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/protos ${CMAKE_CURRENT_SOURCE_DIR}/../MotiveSM/include, ${CMAKE_CURRENT_SOURCE_DIR}/../time_utils/include)
target_link_libraries(tracker_packet_server
    tracker_packet_grpc_proto
)

file(GLOB CLIENT_SOURCES "src/client_*.cxx")
add_library(tracker_packet_client ${CLIENT_SOURCES})
target_include_directories(tracker_packet_client PUBLIC ${TRACKER_PACKET_INCLUDE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/protos)
target_link_libraries(tracker_packet_client
    tracker_packet_grpc_proto
)


