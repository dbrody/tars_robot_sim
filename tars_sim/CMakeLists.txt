cmake_minimum_required(VERSION 2.8.3)
project(tars_sim)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/tars_sim
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
