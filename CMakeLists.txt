cmake_minimum_required(VERSION 2.8)
project(rosgobject)

find_package(catkin)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs std_srvs geometry_msgs rosgraph_msgs
  )

catkin_install_python(PROGRAMS nodes/rxplot nodes/simpleconfig
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
