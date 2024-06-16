cmake_minimum_required(VERSION 3.16)
project(optitrack_bridge CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the output directory for the build executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

# Add the directories to the list of paths to search for include files
include_directories(include)
set(PROJECT_DLL_DEPS "")

# Add the NATNET directories to the list of paths which the compiler should search for libraries
include(cmake/natnet.cmake)

# Add the yaml-cpp library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/include)

# Add the ring_buf header
include_directories(lib/ring_buf/include)

# Add the MotiveSM library
add_subdirectory(lib/MotiveSM)

if (DEFINED ENV{ROS_DISTRO})
    if (ENV{ROS_VERSION} STREQUAL "2")
        message(STATUS "Building for ROS 2")
        # find dependencies
        find_package(ament_cmake REQUIRED)

        # uncomment the following section in order to fill in
        # further dependencies manually.
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)

        add_executable(optitrack_ros2_publisher src/main_ros2.cxx)
        target_link_libraries(optitrack_ros2_publisher MotiveSM)
        ament_target_dependencies(optitrack_ros2_publisher rclcpp std_msgs geometry_msgs)
        install(TARGETS
            optitrack_ros2_publisher
            DESTINATION lib/${PROJECT_NAME})

        install(
            FILES
            ${NATNET_LIB_PATH}
            DESTINATION lib)
        ament_package()
    elseif (ENV{ROS_VERSION} STREQUAL "1")
        message(FATAL_ERROR "Building for ROS 1")
    else ()
        message(FATAL_ERROR "ROS_VERSION must be set to 1 or 2")
    endif ()

else ()
    # Add the search path
    include(cmake/grpc.cmake)
    # Build the protobuf files and generate tp_grpc_proto library
    add_subdirectory(lib/tracker_packet)

    # Add the source directory to the project
    add_subdirectory(src)

    # Add the test directory to the project
    add_subdirectory(examples)
endif ()


add_custom_target(
    copy_dll ALL
    DEPENDS ${PROJECT_DLL_DEPS}
)
