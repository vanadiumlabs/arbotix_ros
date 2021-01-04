cmake_minimum_required(VERSION 3.0.2)
project(arbotix_sensors)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(PROGRAMS bin/ir_ranger.py bin/max_sonar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
