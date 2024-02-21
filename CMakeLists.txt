cmake_minimum_required(VERSION 3.0.2)
project(cansend_generator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS scripts
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    ackermann_msgs
  DEPENDS
)

catkin_install_python(
  PROGRAMS 
  scripts/cansend_generator.py
	scripts/canparsing_imu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

##test