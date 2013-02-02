## Arbotix Drivers

This repository contains the Arbotix ROS drivers, catkinized, and ready for ROS Groovy and newer.

## Changes for Groovy (version 0.8.x)

Several executables are now installed in /opt/ros/groovy/bin allowing you to run them without using rosrun:
 * controllerGUI.py is now arbotix_gui
 * terminal.py is now arbotix_terminal

Other executables have been renamed to alleviate any name collisions:
 * driver.py is now renamed arbotix_driver

