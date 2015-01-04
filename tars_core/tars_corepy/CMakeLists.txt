cmake_minimum_required(VERSION 2.8)

project(tars_corepy)

find_package(rospy REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
	CATKIN_DEPENDS rospy
)

catkin_python_setup()

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
