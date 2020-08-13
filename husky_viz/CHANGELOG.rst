^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
