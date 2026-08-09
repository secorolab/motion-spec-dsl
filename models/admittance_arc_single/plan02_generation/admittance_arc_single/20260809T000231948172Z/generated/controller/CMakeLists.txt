cmake_minimum_required(VERSION 3.16)
project(motion_spec_mj_kdl_target LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED CONFIG)
find_package(coord2b REQUIRED)
find_package(mj_kdl_wrapper 0.2.2 REQUIRED CONFIG)
find_package(tomlplusplus REQUIRED CONFIG)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(bdd_ros2_interfaces REQUIRED)
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
    mj_kdl_wrapper::mj_kdl_wrapper
    orocos-kdl
    tomlplusplus::tomlplusplus
    ${coord2b_LIBRARIES}
    rclcpp::rclcpp
    realtime_tools::realtime_tools
    Threads::Threads
    ${bdd_ros2_interfaces_TARGETS}
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
