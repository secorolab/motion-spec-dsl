cmake_minimum_required(VERSION 3.16)
project(motion_spec_robif2b_target LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED CONFIG)
find_package(robif2b REQUIRED CONFIG)
find_package(tomlplusplus REQUIRED CONFIG)
find_package(coord2b REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

option(MOTION_SPEC_ENABLE_INTROSPECTION "Enable generated runtime introspection logging" OFF)

add_executable(main
    main.cpp
)
target_include_directories(main PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/headers
    ${coord2b_INCLUDE_DIRS}
)
target_link_libraries(main PRIVATE
    orocos-kdl
    tomlplusplus::tomlplusplus
    Threads::Threads
    robif2b::kinova_gen3
    robif2b::robotiq_gripper
    robif2b::robotiq_ft_sensor

    ${coord2b_LIBRARIES}
    rclcpp::rclcpp
    realtime_tools::realtime_tools
    Threads::Threads
    ${sensor_msgs_TARGETS}
)
if(MOTION_SPEC_ENABLE_INTROSPECTION)
    find_package(Protobuf REQUIRED)
    target_sources(main PRIVATE frame_log.pb.cc)
    target_include_directories(main PRIVATE ${Protobuf_INCLUDE_DIRS})
    target_compile_definitions(main PRIVATE MOTION_SPEC_ENABLE_INTROSPECTION)
    target_link_libraries(main PRIVATE Threads::Threads protobuf::libprotobuf)
endif()
target_compile_options(main PRIVATE -Wall -Wextra -ftemplate-depth=2048)
