^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
