cmake_minimum_required(VERSION 2.8.3)
project(tars_sample_controller_py)

find_package(catkin REQUIRED)

# catkin_python_setup()

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
