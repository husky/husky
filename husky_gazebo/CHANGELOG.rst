^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.4 (2020-08-13)
------------------
* Remove support for the Kinect for Xbox 360. We've had the deprecation warning around for a while, so let's finally do it.  Realsense support is in-place as a drop-in replacement that gets added to the top rollbar, just like the old Kinect would have.
* Enable teleop in the Husky simulation
* Removed Paul Bovbel as maintainer.
* Finish adding the simulated realsense to the topbar, add support for the physical realsense. Tidy up some parameters that were copied in last night but not yet configured.
* Mark the Kinect for Xbox 360 as deprecated, start adding support for the Intel Realsense D400 series as a replacement
* Contributors: Chris I-B, Chris Iverach-Brereton, Tony Baltovski

0.4.3 (2020-04-20)
------------------

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

0.3.2 (2019-03-25)
------------------
* Added Z to spawn husky launch
* Contributors: Dave Niewinski

0.3.1 (2018-08-02)
------------------

0.3.0 (2018-04-11)
------------------
* Fixed typo in URLs.
* Corrected the kinect scan topic name so it matches the laser scanner topic name.
* description.launch should be launched only once when the flag multimaster is false.
  This commit fixes `#47 <https://github.com/husky/husky/issues/47>`_.
  Also delete the urdf subdirectory from the installation list of the package husky_gazebo
  since the directory doesn't exist.
  HOW BUILT
  $ catkin_make_isolated --install --use-ninja
  HOW VERIFIED
  $ roslaunch husky_gazebo husky_playpen.launch
  $ roslaunch husky_viz view_robot.launch
  Verify that the LiDAR is present on the gazebo husky and works as expected.
  Change-Id: I8797d561489250417dc8fe2b49d958993ca7949c
  Signed-off-by: Wei Ren <renwei@smartconn.cc>
* Remove defunct email address
* Updated maintainers.
* Temp commit
* Add interface definitions
* Update bringup for multirobot
* Purge more UR; Implement urdf_extras
* Update URDF for multirobot
* Move packages into monorepo for kinetic; strip out ur packages
* Contributors: Farzad Niroui, Paul Bovbel, Tony Baltovski, Wei Ren

0.2.6 (2016-10-26)
------------------
* spawn_husky.launch: enable to use custom controller files, i.e effort controller
* spawn_husky.launch: support argument to set urdf file and initial pose
* Contributors: Kei Okada

0.2.5 (2015-12-31)
------------------
* Removed duplicate SICK laser plugin in husky_gazebo, since husky_description already contains a SICK laser plugin from the lms package.
* Contributors: Peiyi Chen

0.2.4 (2015-07-08)
------------------
* Add pointcloud to laserscan config for simulated kinect
* Contributors: Paul Bovbel

0.2.3 (2015-04-13)
------------------
* Fix conflict with underlay
  When using -z check, underlayed instances of husky_gazebo would override overlays.
* Contributors: Paul Bovbel

0.2.2 (2015-04-08)
------------------
* Reduce physics update rate
* Integrate husky_customization workflow
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------
* Fix package urls
* Add missing dependency
* Contributors: Paul Bovbel

0.2.0 (2015-03-23)
------------------
* Refactor URDF
* Add UR5 and Kinect simulation
* Contributors: Paul Bovbel, TheDash

0.1.4 (2015-02-11)
------------------
* Add missing dependency
* Contributors: Paul Bovbel

0.1.3 (2015-02-06)
------------------
* Fix installing
* Contributors: Paul Bovbel

0.1.2 (2015-01-30)
------------------
* Update authors
* Add missing dependency
* Reduce sensor range
* Contributors: Paul Bovbel

0.1.1 (2015-01-14)
------------------
* Remove multirobot prefixing, experiment later
* Contributors: Paul Bovbel

0.1.0 (2015-01-13)
------------------
* Major refactor for indigo:
  * All gazebo plugins moved to urdf/description.gazebo.xacro from husky_description
  * Ported to ros_control
* Contributors: James Servos, Mike Purvis, Paul Bovbel, Prasenjit Mukherjee, y22ma

0.0.3 (2013-11-01)
------------------

0.0.2 (2013-09-30)
------------------
* added package installation rules

0.0.1 (2013-09-29)
------------------
* Initial release for Hydro.
