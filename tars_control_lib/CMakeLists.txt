cmake_minimum_required(VERSION 2.8)

project(tars_control_lib)

find_package(roscpp REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES tars_control_lib
	CATKIN_DEPENDS roscpp
)

include_directories(include ${roscpp_INCLUDE_DIRS})

add_library(tars_control_lib src/tars_control_lib.cpp)
target_link_libraries(tars_control_lib ${roscpp_LIBRARIES})
