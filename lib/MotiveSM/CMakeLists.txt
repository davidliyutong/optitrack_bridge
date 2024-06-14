file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx")

add_library(MotiveSM ${SOURCES})
target_include_directories(MotiveSM PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(MOTIVE_SM_INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include" PARENT_SCOPE)
target_link_libraries(
    MotiveSM
    ${NATNET_LIB} yaml-cpp
)