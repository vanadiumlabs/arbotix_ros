^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package arbotix_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2020-12-29)
-------------------
* Update all python shebangs to python3 + rosdep dependency (`#48 <https://github.com/vanadiumlabs/arbotix_ros/issues/48>`_)
  Co-authored-by: Murat Calis <mc@pirate-robotics.net>
* arbotix_python for ROS Noetic (`#46 <https://github.com/vanadiumlabs/arbotix_ros/issues/46>`_)
  Migrated arbotix_python to work with ROS Noetic
  Co-authored-by: Murat Calis <mc@pirate-robotics.net>
* Merge pull request `#31 <https://github.com/vanadiumlabs/arbotix_ros/issues/31>`_ from corot/serial_reconnect
  Allow runtime connection/disconnection to/from ArbotiX board
* Merge pull request `#29 <https://github.com/vanadiumlabs/arbotix_ros/issues/29>`_ from croesmann/indigo-devel
  Allow cancelling the FollowJointTrajectoryAction during execution
* Merge pull request `#33 <https://github.com/vanadiumlabs/arbotix_ros/issues/33>`_ from corot/issue_26
  Issue `#26 <https://github.com/vanadiumlabs/arbotix_ros/issues/26>`_ implementation: enable/relax services on ServoController class
* Issue `#26 <https://github.com/vanadiumlabs/arbotix_ros/issues/26>`_ implementation: enable/relax services to the ServoController
  class, so you don't need to call service on each servo
* Close serial port only if not fake
* Allow runtime connection/disconnection to/from ArbotiX board
* Minor formatting fix
* Fixed formatting issues
* the follow joint trajectory action can now be canceled during execution
* Fix syntax
* Merge pull request `#25 <https://github.com/vanadiumlabs/arbotix_ros/issues/25>`_ from corot/indigo-devel
  Implement issue https://github.com/vanadiumlabs/arbotix_ros/issues/24:
* leng -> length
* Implement issue https://github.com/vanadiumlabs/arbotix_ros/issues/24:
  Allow 16 bit values on arbotix_msgs/Analog messages, but assume 8 bits
  by default
* Contributors: Christoph Rösmann, Jorge Santos Simón, Michael Ferguson, calismurat, corot

0.10.0 (2014-07-14)
-------------------
* Set queue_size=5 on all publishers
* Contributors: Jorge Santos

0.9.2 (2014-02-12)
------------------

0.9.1 (2014-01-28)
------------------
* set velocity when in sim/fake mode
* Added set_speed service to servo controller
* Added 'set spd' option to arbotix_terminal

0.9.0 (2013-08-22)
------------------
* Add new enable service
* remove roslib manifest loading
* remove old dynamixels block, closes `#6 <https://github.com/vanadiumlabs/arbotix_ros/issues/6>`_
* Warn of extra joints in joint trajectory, but only fail when missing a joint we control

0.8.2 (2013-03-28)
------------------

0.8.1 (2013-03-09)
------------------
* fix depend for proper release

0.8.0 (2013-02-21)
------------------
* fix follow controller issues with zeros in header timestamps, cleanup logging a bit
* import drivers and catkinize
