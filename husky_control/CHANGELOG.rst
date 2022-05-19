^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2022-05-19)
------------------
* [husky_control] Disabled imu_filter_madgwick for now.
* Contributors: Tony Baltovski

1.0.7 (2022-05-19)
------------------
* [husky_control] Fixed joy device param.
* Renamed all launch files to *.launch.py.
* Contributors: Tony Baltovski

1.0.6 (2022-05-18)
------------------
* Added searching for left and right joints rather than assuming order.
* Contributors: Tony Baltovski

1.0.5 (2022-05-05)
------------------
* [husky_control] Fixed deprecated warnings and minor clean up.
* Split teleop launch into two files since simulation doesn't need actual joystick and will spam warmings.
* [husky_control] Removed dupilcate config.
* [husky_control] Added IMU filter.
* Revamped tele-op launch.
* [husky_control] Cleaned up control_launch.py.
* [husky_control] Re-added interactive_marker_twist_server and sorted depends in-order.
* Contributors: Tony Baltovski

1.0.4 (2022-03-15)
------------------
* Merge pull request `#191 <https://github.com/husky/husky/issues/191>`_ from StoglRobotics-forks/gazebo-sim-integration-fixes
  Gazebo sim integration fixes
* Contributors: Tony Baltovski

1.0.3 (2021-11-30)
------------------

1.0.2 (2021-11-16)
------------------
* Correct name of joint state broadcaster (controller) does not exist anymore.
* Contributors: Denis Štogl

1.0.1 (2021-11-12)
------------------

1.0.0 (2021-11-07)
------------------
* Initial Gazebo Classic changes.
* [husky_control] Added basic localization config.
* [husky_control] Disabled interactive_marker_twist_server for now.
* Removed missing packages in ROS2.
* [husky_control] Removed multimaster_launch.
* [husky_control] Added teleop launch.
* [husky_control] Update control rate to 10Hz.
* Updates to use ros2_control.
* [husky_control] Updated CMakeLists.txt.
* Initial attempt at ros2_control.
* Add the link_name parameter to fix the interactive markers in rviz
* Contributors: Chris Iverach-Brereton, Tony Baltovski

0.4.4 (2020-08-13)
------------------
* clearer wording
* change if to unless
* added env var and if-statement to disable robot ekf
* Remove support for the Kinect for Xbox 360. We've had the deprecation warning around for a while, so let's finally do it.  Realsense support is in-place as a drop-in replacement that gets added to the top rollbar, just like the old Kinect would have.
* Removed Paul Bovbel as maintainer.
* Finish adding the simulated realsense to the topbar, add support for the physical realsense. Tidy up some parameters that were copied in last night but not yet configured.
* Contributors: Chris I-B, Chris Iverach-Brereton, Jose Mastrangelo, Tony Baltovski

0.4.3 (2020-04-20)
------------------
* Update the udev rules to map the controllers to appropriate symlinks instead of relying on device enumeration to save us
* Remove the device override for the PS4 controller since we pair with bluez now (which maps the device to /dev/input/js0)
* Fix the filename in the launch fike
* Make the Logitech controller config file explicit. Add ascii-art controllers to label the axes to make configuration easier
* Contributors: Chris I-B, Chris Iverach-Brereton

0.4.2 (2019-12-11)
------------------

0.4.1 (2019-09-30)
------------------
* Added envar for joy device.
* Contributors: Tony Baltovski

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
* Updated default controller to be PS4.  Can be set back to logitech (legacy) by setting HUSKY_LOGITECH environment variable
* Contributors: Dave Niewinski

0.3.0 (2018-04-11)
------------------
* Updated all package versions to 0.2.6.
* Made multimaster not come up by default in husky_control
* [husky_control] Fixed typo.
* Updated the rolling window size for more responsive control
* Fixed typo in URLs.
* Added dependency on husky_description to husky_control/package.xml
* Remove defunct email address
* Updated maintainers.
* Added more details to the config_extras workflow.
* Temp commit
* Add interface definitions
* Revert "Remove twist_mux config."
  (cherry picked from commit 4ae73877d0d3b0db8e6bc6be18f0648ea310d372)
* Update bringup for multirobot
* Purge more UR; Implement urdf_extras
* Update URDF for multirobot
* Remove twist_mux config.
* Replace twist-mux
* Contributors: Administrator, Dave Niewinski, Paul Bovbel, Peiyi Chen, TheDash, Tony Baltovski

0.2.7 (2015-12-31)
------------------
* Update localization.yaml
* Update localization.yaml
* Remapping the move_base topic to be compatible with cpr autonomy core.
* Contributors: Peiyi Chen, Tom Moore

0.2.6 (2015-07-08)
------------------
* Added angular_scale_turbo to teleop.config.
* Move interactive marker launch from teleop into control launch file
* Added fix for ur5 arm in gazebo
* Contributors: Paul Bovbel, Devon Ash, Tony Baltovski

0.2.5 (2015-04-16)
------------------

0.2.4 (2015-04-13)
------------------

0.2.3 (2015-04-08)
------------------


0.2.2 (2015-03-23)
------------------
* Fix package urls
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------
* Update control params with base_link
* Contributors: Paul Bovbel

0.2.0 (2015-03-23)
------------------
* Add UR5 arm simulation control config
* Contributors: Paul Bovbel, Devon Ash

0.0.4 (2015-02-12)
------------------
* Namespace fixes
* Contributors: Paul Bovbel

0.0.3 (2015-02-06)
------------------

* Update website
* Add author
* Get rid of chassis_link, switch to base_footprint and base_link
* Turn on 2d mode; future proof robot_localization parameters
* Refactor configuration files into modules
* Re-enable IMU orientation fusion
* Contributors: Paul Bovbel

0.0.2 (2015-01-16)
------------------
* Use odom position for ekf
* Update wheel separation multiplier for slippage
* Restore teleop twist joy
* Set 2D mode, and add move_base cmd channel
* Contributors: Paul Bovbel

0.0.1 (2015-01-12)
------------------
* Initial development of husky_control for Husky indigo release
* Contributors: Paul Bovbel
