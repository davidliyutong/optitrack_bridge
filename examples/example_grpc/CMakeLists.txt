# Define the server executable
add_executable(example_grpc_server example_grpc_server.cxx)
target_include_directories(example_grpc_server PUBLIC "../../lib/tracker_packet/include" "../../lib/tracker_packet/protos")
target_link_libraries(example_grpc_server
    tracker_packet_server
    absl::flags_parse
)

# Define the server executable
add_executable(example_grpc_client example_grpc_client.cxx)
target_include_directories(example_grpc_client PUBLIC "../../lib/tracker_packet/include"  "../../lib/tracker_packet/protos")
target_link_libraries(example_grpc_client
    tracker_packet_client
    absl::flags_parse
)