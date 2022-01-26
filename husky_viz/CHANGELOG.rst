^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2022-01-18)
------------------
* Add rqt directory to CMakeLists
* Add rqt_gui as run_depend
* Check launch file only if testing
  When building husky_control, husky_description, husky_navigation or
  husky_viz without tests, CMake fails as it does not find
  `catkin_run_tests_target` command. This patch adds conditions to fix
  this problem.
* Added view_diagnostics.launch
* Contributors: Alexandre Iooss, Luis Camero, luis-camero

0.6.0 (2021-09-28)
------------------

0.5.1 (2021-09-16)
------------------
* Remove the need to explicitly specify the laser_enabled, realsense_enabled, and urdf_extras arguments; use the envars to make it easier to simulate customized robots & use the moveit setup assistant.
* Add VLP16, secondary LMS1xx support (`#164 <https://github.com/husky/husky/issues/164>`_)
  * Minimal refactor to add VLP16 + secondary LMS1xx support. Update defaults for the laser_enabled and realsense_enabled args to refer to the underlying envars to improve consistency when launching simulations. Modify the sensor bar to allow it to be positioned in the center by default, but with configurable xyz and rpy offsets
  * Add the new run dependencies
  * Remove the prefix's trailing underscore in the vlp16 mount to make it consistent. Fix an inconsistent envar for the sensor arch, add an arg to explicitly enable it, to stay internally consistent with the rest of Husky.
  * Fix the envars; its just HUSKY_LMS1XX, not HUSKY_LASER_LMS1XX
  * Revert to enabling the main laser by default in the simulations, add the velodyne_gazebo_plugins dependency
* Contributors: Chris I-B

0.5.0 (2021-08-23)
------------------

0.4.4 (2020-08-13)
------------------
* Remove support for the Kinect for Xbox 360. We've had the deprecation warning around for a while, so let's finally do it.  Realsense support is in-place as a drop-in replacement that gets added to the top rollbar, just like the old Kinect would have.
* Removed Paul Bovbel as maintainer.
* Mark the Kinect for Xbox 360 as deprecated, start adding support for the Intel Realsense D400 series as a replacement
* Contributors: Chris I-B, Chris Iverach-Brereton, Tony Baltovski

0.4.3 (2020-04-20)
------------------
* [husky_viz] Removed joint_state_publisher since joint_state_publisher_gui is generating the same data.
* Fix a deprecation warning about the joint state publisher's gui
* Contributors: Chris Iverach-Brereton, Tony Baltovski

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

0.3.1 (2018-08-02)
------------------
* Updated dependencies for robot_state_publisher
* Updated view_model.launch to include a robot_state_publisher since it was moved to control.launch
* Contributors: Dave Niewinski

0.3.0 (2018-04-11)
------------------
* Updated all package versions to 0.2.6.
* Fixed typo in URLs.
* Remove defunct email address
* Updated maintainers.
* Update URDF for multirobot
* Move packages into monorepo for kinetic; strip out ur packages
* Contributors: Paul Bovbel, Tony Baltovski

0.2.2 (2015-04-08)
------------------
* Toggle viz settings
* Integrate husky_customization workflow
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------
* Update robot visualization
* Enable arm visualization by default
* Fix package urls
* Contributors: Paul Bovbel

0.2.0 (2015-03-23)
------------------


0.1.1 (2015-02-06)
------------------
* Update authors and web
* Add IMU viz dependency; Update rviz presets
* Restore interactive marker
* Contributors: Paul Bovbel

0.1.0 (2015-01-14)
------------------
* Update visualizations for indigo release
* update maintainer and dependencies
* Remove husky_interactive_markers, switch to generic.
* Add front_laser arg to view_model launcher.
* Default fixed frame for the model viewer should be base_link.
* viewing more options for navigation.rviz
* Contributors: Mike Purvis, Paul Bovbel, Prasenjit Mukherjee

0.0.3 (2013-10-04)
------------------
* Set version requirement for description pkg.
* Navigation view launch file and rviz file

0.0.2 (2013-09-29)
------------------
* Add interactive_markers pkg to dependencies.
* Add rviz files and launch files for view_robot (odom view + interactive marker control)
* Add roslaunch file check.

0.0.1 (2013-09-11)
------------------
* Working view_model.launch.
* Catkinize package, add install targets.
* Split off separate husky_viz package.
* Move models directory up to root
