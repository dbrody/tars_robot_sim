cmake_minimum_required(VERSION 2.8)

project(tars_corecpp)

find_package(roscpp REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES tars_corecpp
	CATKIN_DEPENDS roscpp
)

include_directories(include ${roscpp_INCLUDE_DIRS})

add_library(tars_corecpp src/tars_corecpp.cpp)
target_link_libraries(tars_corecpp ${roscpp_LIBRARIES})
