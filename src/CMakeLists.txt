# Add executable target with your source files
add_executable(optitrack_bridge main.cxx)

# Link the libraries with the executable
target_link_libraries(
    optitrack_bridge
    tracker_packet_server ${NATNET_LIB} yaml-cpp absl::flags_parse MotiveSM
)