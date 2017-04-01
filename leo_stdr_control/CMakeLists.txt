cmake_minimum_required(VERSION 2.8.3)
project(leo_stdr_control)

find_package(catkin_simple REQUIRED)
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
cs_add_executable(leo_lidar_alarm src/leo_lidar_alarm.cpp)

cs_add_executable(leo_reactive_commander src/leo_reactive_commander.cpp)

cs_install()
cs_export()



