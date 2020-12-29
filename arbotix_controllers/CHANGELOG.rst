^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package arbotix_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2020-12-29)
-------------------
* Update all python shebangs to python3 + rosdep dependency (`#48 <https://github.com/vanadiumlabs/arbotix_ros/issues/48>`_)
  Co-authored-by: Murat Calis <mc@pirate-robotics.net>
* Merge pull request `#22 <https://github.com/vanadiumlabs/arbotix_ros/issues/22>`_ from corot/indigo-devel
  roslib.load_manifest should not be used on catkin packages
* roslib.load_manifest should not be used on catkin packages according to http://wiki.ros.org/PyStyleGuide
* Contributors: Jorge Santos, Michael Ferguson, calismurat

0.10.0 (2014-07-14)
-------------------
* Set queue_size=5 on all publishers
* Check if command exceeds opening limits
* Contributors: Jorge Santos

0.9.2 (2014-02-12)
------------------
* cleanup gripper controllers, mark deprecations
* Contributors: Michael Ferguson

0.9.1 (2014-01-28)
------------------

0.9.0 (2013-08-22)
------------------
* fix joint_states subscriber
* fix name of singlesided model
* add new gripper action controller

0.8.2 (2013-03-28)
------------------

0.8.1 (2013-03-09)
------------------

0.8.0 (2013-02-21)
------------------
* import drivers and catkinize
