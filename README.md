## Arbotix Drivers

This repository contains the Arbotix ROS drivers, catkinized, and ready for ROS Groovy and newer.

## Changes for version 0.11.x
Controller will now break when goal is reached.  
Added support for PhantomX Pincher.   
   To use the PhantomX Pincher, set environment variable "TURTLEBOT_ARM1" to pincher. You will need turtlebot_arm version 0.4.0 or higher for PhantomX Pincher 

## Changes for Groovy (version 0.8.x)

Several executables are now installed in /opt/ros/groovy/bin allowing you to run them without using rosrun:
 * controllerGUI.py is now arbotix_gui
 * terminal.py is now arbotix_terminal

Other executables have been renamed to alleviate any name collisions:
 * driver.py is now renamed arbotix_driver

