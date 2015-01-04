cmake_minimum_required(VERSION 2.8.3)
project(tars_controller_nn)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  rostime
  std_msgs
  tars_corecpp
)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${roscpp_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs
)



set(tars_controller_nn_node_SRCS
	src/tars_controller_nn.cpp
	src/neuron.cpp
	src/tars_controller_nn_frame.cpp
)

set(tars_controller_nn_node_HDRS
	include/tars_controller_nn/tars_controller_nn_frame.h
)

qt4_wrap_cpp(tars_controller_nn_node_MOCS ${tars_controller_nn_node_HDRS})



add_executable(tars_controller_nn_node ${tars_controller_nn_node_SRCS} ${tars_controller_nn_node_MOCS})
target_link_libraries(tars_controller_nn_node ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})



install(TARGETS tars_controller_nn_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


