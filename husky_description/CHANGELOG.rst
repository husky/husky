^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.6 (2023-01-16)
------------------

0.6.5 (2022-11-25)
------------------
* Removed old bracket
* Updated brackets
* Change the reference heading of the GPS plugin to be east so the coordinate system is ENU
* Rename DATUM\_* to GAZEBO_WORLD\_*. We're separating the datum from the world's origin, and this keeps envars from clobbering each other
* LMS1XX Tower EnvVar (`#250 <https://github.com/husky/husky/issues/250>`_)
  * Added second velodyne description
  * Added environment variable to remove the LMS1XX mount
  * Removed console output
* Use DATUM_LAT and DATUM_LON envars to override the default reference lat/lon for the Gazebo GPS plugin. This addresses a compatibility issue with outdoor nav simulations
* Contributors: Chris Iverach-Brereton, Luis Camero, luis-camero

0.6.4 (2022-06-16)
------------------
* Fixes for velodyne prefix
* PACS Brackets (`#238 <https://github.com/husky/husky/issues/238>`_)
  * Added STL models for brackets
  * Added brackets to URDF
  * Added bracket environment variables
* Fixed issues with find
* Added README with all Environment Variables
* Duplicate URDF for every standard sensor
* Added fath pivot mount as dependency
* Added URDF of Blackfly on top of fath_pivot_mount
* Added prefix to name parameter to differentiate between primary and secondary
* Changed ENABLE to ENABLED
* Changed HUSKY_FULL_RISER to HUSKY_FULL_RISER_LEVEL
* Added PACS top plate and mounts to decorations
* Added PACS urdf definitions
* Added PACS meshes
* Contributors: Luis Camero, luis-camero

0.6.3 (2022-05-16)
------------------
* Added Blackfly
* Added blackfly description dependancy
* Realsense will no longer add the Sensor Arch (`#220 <https://github.com/husky/husky/issues/220>`_)
  * Add parent link env-var to imu
  * Sensor Arch is only enabled by HUSKY_SENSOR_ARCH, changed default realsense mount frame to
* Contributors: Luis Camero, luis-camero

0.6.2 (2022-02-15)
------------------
* Added Wibotic mesh and STL
* Bump CMake version to avoid CMP0048 warning.
* Contributors: Luis Camero, Tony Baltovski

0.6.1 (2022-01-18)
------------------
* Fixed error in URDF
* Added Hokuyo
* Check launch file only if testing
  When building husky_control, husky_description, husky_navigation or
  husky_viz without tests, CMake fails as it does not find
  `catkin_run_tests_target` command. This patch adds conditions to fix
  this problem.
* Revert changes to mount_base_link to preserve use of origin argument; instead, simply created an additional plate link that sits between the supports and VLP16
* Create visual geometry of vlp16_mount_base_link
* Update mount support dimensions and spacing to better represent real-world measurements
* [husky_description] Fixed malformed STL warning for top_plate.stl.
* Contributors: Alexandre Iooss, Luis Camero, Tony Baltovski, jyang-cpr

0.6.0 (2021-09-28)
------------------

0.5.1 (2021-09-16)
------------------
* cpr urdf extras
* Remove the need to explicitly specify the laser_enabled, realsense_enabled, and urdf_extras arguments; use the envars to make it easier to simulate customized robots & use the moveit setup assistant.
* Update intel_realsense.urdf.xacro
  modify image format in sim to avoid log warn spam
* Add HUSKY\_{FRONT|REAR}_BUMPER envars we can use to completely turn off the front & rear bumpers.  This is requested to make integration of the wireless charging docks easier
* Add VLP16, secondary LMS1xx support (`#164 <https://github.com/husky/husky/issues/164>`_)
  * Minimal refactor to add VLP16 + secondary LMS1xx support. Update defaults for the laser_enabled and realsense_enabled args to refer to the underlying envars to improve consistency when launching simulations. Modify the sensor bar to allow it to be positioned in the center by default, but with configurable xyz and rpy offsets
  * Add the new run dependencies
  * Remove the prefix's trailing underscore in the vlp16 mount to make it consistent. Fix an inconsistent envar for the sensor arch, add an arg to explicitly enable it, to stay internally consistent with the rest of Husky.
  * Fix the envars; its just HUSKY_LMS1XX, not HUSKY_LASER_LMS1XX
  * Revert to enabling the main laser by default in the simulations, add the velodyne_gazebo_plugins dependency
* Add the ability to add the sensor bar with an envar without adding the realsense.  Add the sensor bar height as another arg + envar, fix the URDF when the 300mm sensorbar is enabled.
* Contributors: Chris I-B, Ebrahim Shahrivar, vamshi konduri

0.5.0 (2021-08-23)
------------------
* Update husky.urdf.xacro (`#169 <https://github.com/husky/husky/issues/169>`_)
  Fix Failed to build tree: child link [base_laser_mount] of joint [laser_mount_joint] not found error.
  As found on https://answers.ros.org/question/354219/failed-to-build-tree-child-link-base_laser_mount-of-joint-laser_mount_joint-not-found/
* Contributors: Guido Sanchez

0.4.4 (2020-08-13)
------------------
* Remove support for the Kinect for Xbox 360. We've had the deprecation warning around for a while, so let's finally do it.  Realsense support is in-place as a drop-in replacement that gets added to the top rollbar, just like the old Kinect would have.
* Changed intertial to inertial
  fixed a minor typo
* Removed Paul Bovbel as maintainer.
* Fix the warnings the ROS buildfarm was giving for Melodic
* Add support for some environment variables to override realsense defaults
* Use the STL from realsense2_description, rotated as necessary. Add realsense2_description to the dependencies
* Refactor so that the sensor bar only gets added once if either the realsense OR the kinect is enabled. Adding both will still cause issues because they'll mount to the same point on the bracket, but at least the URDF won't fail.
* Finish adding the simulated realsense to the topbar, add support for the physical realsense. Tidy up some parameters that were copied in last night but not yet configured.
* Mark the Kinect for Xbox 360 as deprecated, start adding support for the Intel Realsense D400 series as a replacement
* Contributors: Cedric Martens, Chris I-B, Chris Iverach-Brereton, Tony Baltovski

0.4.3 (2020-04-20)
------------------
* Fixed GazeboRosControlPlugin missing error
* Contributors: lerolynn

0.4.2 (2019-12-11)
------------------

0.4.1 (2019-09-30)
------------------

0.4.0 (2019-08-01)
------------------

0.3.4 (2019-08-01)
------------------

0.3.3 (2019-04-18)
------------------
* Fixed bumper extensions, cleaned up collision meshes
* Contributors: Dave Niewinski

0.3.2 (2019-03-25)
------------------
* Added some additional frames on the top plates and an environment variable for diabling the user rails
* Added env var to allow a 7cm forward bumper extension (`#92 <https://github.com/husky/husky/issues/92>`_)
  * Added env var to allow for extendable front bumper
  * Fix weird spacing
  * Uploaded bumper extension meshes
  * Allowed for different lengths of bumper extensions
* Contributors: Dave Niewinski, Guy Stoppi

0.3.1 (2018-08-02)
------------------
* Removed unnecessary dae objects and duplicate vertices
* Contributors: Dave Niewinski

0.3.0 (2018-04-11)
------------------
* Updated all package versions to 0.2.6.
* Added a large top plate (used for waterproofing upgrade and UR5 upgrade) and an environment variable for controlling it HUSKY_LARGE_TOP_PLATE
* changed Husky wheel radius, a Husky outdoor tire is 13 inchs (0.3302m)
* [husky_description] Updated inertial parameters.
* [husky_description] Fixed depreciated syntax.
* Remove defunct email address
* Updated maintainers.
* Changes for xacro updates in kinetic.
* Add interface definitions
* Purge more UR; Implement urdf_extras
* Update URDF for multirobot
* Move packages into monorepo for kinetic; strip out ur packages
* wheel.urdf.xacro: swap iyy, izz inertias
  Fixes `#34 <https://github.com/husky/husky/issues/34>`_.
* Contributors: Dave Niewinski, Martin Cote, Paul Bovbel, Steven Peters, Tony Baltovski

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
* Port stl to dae format, removing material/gazebo colours
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
