# CMake rules for the NatNet SDK

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/NatNetSDK/include)
if (WIN32)
    set(NATNET_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/NatNetSDK/lib/x64/NatNetLib.dll)
    find_library(NATNET_LIB NAMES NatNetLib PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/NatNetSDK/lib/x64)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/NatNetSDK/lib/x64)
    add_custom_command(
        OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/NatNetLib.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${NATNET_LIB_PATH}
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/NatNetLib.dll
    )
    list(APPEND PROJECT_DLL_DEPS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/NatNetLib.dll)
elseif (UNIX AND NOT APPLE)
    set(NATNET_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/NatNetSDK/lib/libNatNet.so)
    find_library(NATNET_LIB NAMES libNatNet.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/NatNetSDK/lib)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/NatNetSDK/lib)
    add_custom_command(
        OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libNatNet.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${NATNET_LIB_PATH}
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libNatNet.so
    )
elseif (APPLE)
    # display error message and exit
    message(FATAL_ERROR "Apple is not supported")
endif ()
message(STATUS "NATNET_LIB_PATH: ${NATNET_LIB_PATH}")