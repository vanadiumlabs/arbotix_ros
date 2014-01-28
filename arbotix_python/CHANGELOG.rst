^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package arbotix_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
