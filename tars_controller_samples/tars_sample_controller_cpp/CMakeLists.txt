cmake_minimum_required(VERSION 2.8.3)
project(tars_sample_controller_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rostime
  std_msgs
  tars_corecpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs
)


set(tars_sample_controller_cpp_node_SRCS
	src/tars_sample_controller_cpp.cpp
)

set(tars_sample_controller_cpp_node_HDRS
	include/tars_sample_controller_cpp/tars_sample_controller_cpp.h
)


add_executable(tars_sample_controller_cpp_node ${tars_sample_controller_cpp_node_SRCS} ${tars_sample_controller_cpp_node_MOCS})
target_link_libraries(tars_sample_controller_cpp_node ${catkin_LIBRARIES})

install(TARGETS tars_sample_controller_cpp_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


