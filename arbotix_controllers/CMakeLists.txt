cmake_minimum_required(VERSION 3.0.2)
project(arbotix_controllers)

find_package(catkin REQUIRED)
catkin_package()

install(
  PROGRAMS
    bin/gripper_controller
    bin/one_side_gripper_controller.py
    bin/parallel_gripper_controller.py
    bin/parallel_single_servo_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
