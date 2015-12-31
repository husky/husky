^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.7 (2015-12-31)
------------------
* Fixed indent.
* Added Sick LMS1XX URDF.
* Contributors: Tony Baltovski

0.2.6 (2015-07-08)
------------------
* Adjust Kinect angle so it doesn't hit top plate
* Contributors: Paul Bovbel

0.2.5 (2015-04-16)
------------------
* Add standard mount for lms1xx
* Contributors: Paul Bovbel

0.2.4 (2015-04-13)
------------------
* Add argument to enable/disable top plate
* Fix sensor arch name
* Fix conflict with underlay
  When using -z check, underlayed instances of husky_gazebo would override overlays.
* Update top plate model
* Contributors: Paul Bovbel

0.2.3 (2015-04-08)
------------------
* Integrate husky_customization workflow
* Disable all accessories by default
* Contributors: Paul Bovbel

0.2.2 (2015-03-23)
------------------
* Fix package urls
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------
* Port *.stl to *.dae format, removing material/gazebo colours
* Make base_footprint a child of base_link
* Contributors: Paul Bovbel

0.2.0 (2015-03-23)
------------------
* Add Kinect, UR5 peripherals
* Contributors: Paul Bovbel, Devon Ash

0.1.2 (2015-01-30)
------------------
* Update maintainers and description
* Get rid of chassis_link, switch to base_footprint and base_link
* Switch to NED orientation for UM6 standard package
* Contributors: Paul Bovbel

0.1.1 (2015-01-14)
------------------
* Remove multirobot changes, experiment later
* Contributors: Paul Bovbel

0.1.0 (2015-01-13)
------------------
* Major refactor for indigo release:
  * base_link is now located on the ground plane, while chassis_link
  * refactored joint names for consistency with Jackal and Grizzly for ros_control
  * moved plugins requiring gazebo dependencies to husky_gazebo (imu, gps, lidar, ros_control)
  * initial prefixing for multirobot
* Contributors: Alex Bencz, James Servos, Mike Purvis, Paul Bovbel, Prasenjit Mukherjee, y22ma

0.0.2 (2013-09-30)
------------------
* Renamed /models folder to /meshes to follow the convention of other gazebo simulation packages.
* Changed the base.urdf.xacro to use base_footprint as the parent frame. For some reason, the new Gazebo paints all parts the same color as base_link when base_link is the parent.

0.0.1 (2013-09-11)
------------------
* Move to model-only launchfile.
* Catkinize package, add install targets.
* husky_description moved up to repository root.
