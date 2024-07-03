cmake_minimum_required(VERSION 3.0.2)
project(cansend_generator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ackermann_msgs
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy ackermann_msgs roscpp std_msgs sensor_msgs  # Include roscpp and sensor_msgs here
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the C++ executable
add_executable(imu_parsing src/imu_parsing.cpp)

# Link the executable with catkin libraries
target_link_libraries(imu_parsing
  ${catkin_LIBRARIES}
)

# Install the executable
install(TARGETS imu_parsing
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/cansend_generator.py
  scripts/joy_detect.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

