^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.7 (2022-05-19)
------------------
* Renamed all launch files to *.launch.py.
* Contributors: Tony Baltovski

1.0.6 (2022-05-18)
------------------

1.0.5 (2022-05-05)
------------------
* Updated package versions for un-released packages.
* [husky_bringup] Disabled gps bringup for now since nmea_navsat_driver isn't released.
* Initial accessories launch file.
* Contributors: Tony Baltovski

0.4.4 (2020-08-13)
------------------
* Set default for optenv HUSKY_MAG_CONFIG
* Removed env-hooks
* Removed Paul Bovbel as maintainer.
* Add support for some environment variables to override realsense defaults
* Sort the dependencies alphabetically
* Finish adding the simulated realsense to the topbar, add support for the physical realsense. Tidy up some parameters that were copied in last night but not yet configured.
* Mark the Kinect for Xbox 360 as deprecated, start adding support for the Intel Realsense D400 series as a replacement
* Contributors: Chris I-B, Dave Niewinski, Tony Baltovski

0.4.3 (2020-04-20)
------------------
* Add the other product ID for the PS4 controller to the udev rule
* Update the udev rules to map the controllers to appropriate symlinks instead of relying on device enumeration to save us
* Contributors: Chris I-B

0.4.2 (2019-12-11)
------------------
* [husky_bringup] Installed udev rule for Logitech controller.
* Contributors: Tony Baltovski

0.4.1 (2019-09-30)
------------------
* [husky_bringup] Enabled using MagnenticField message for UM6 and UM7 since imu_filter_madgwick now uses it by default.
* Added Udev rule for Logitech joy. (`#116 <https://github.com/husky/husky/issues/116>`_)
* Contributors: Tony Baltovski

0.4.0 (2019-08-01)
------------------

0.3.4 (2019-08-01)
------------------
* Properly support GX5.
* Contributors: Dave Niewinski

0.3.3 (2019-04-18)
------------------

0.3.2 (2019-03-25)
------------------
* [husky_bringup] Disabled the use of magnetic field msgs in imu_filter_madgwick.
* Contributors: Tony Baltovski

0.3.1 (2018-08-02)
------------------
* Renamed udev so it is installed
* Contributors: Dave Niewinski, Tony Baltovski

0.3.0 (2018-04-11)
------------------
* Remove defunct email address
* Re-added microstrain_3dmgx2_imu as run  dependency.
* Re-added imu_transformer and um7 as run dependencies.
* Updated maintainers.
* Changed the name of robot_upstart job to ros.
* Added the UM6 as a run dep.
* Purge more UR; Implement urdf_extras
* Move packages into monorepo for kinetic; strip out ur packages
* Contributors: Paul Bovbel, Tony Baltovski

0.2.6 (2016-10-03)
------------------
* Adding support for the UM7 IMU.
* Added new ur_modern_driver
* Added param for laser frame_id.
* Contributors: TheDash, Tony Baltovski

0.2.5 (2015-12-31)
------------------

0.2.4 (2015-07-08)
------------------
* Fix laser path
* Contributors: Paul Bovbel

0.2.3 (2015-04-08)
------------------
* Integrate husky_customization workflow
* Contributors: Paul Bovbel

0.2.2 (2015-03-23)
------------------
* Fix package urls
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------

0.2.0 (2015-03-23)
------------------
* Add UR5 bringup
* Contributors: Paul Bovbel, Devon Ash

0.1.2 (2015-02-12)
------------------
* Namespace fixes
* Contributors: Paul Bovbel

0.1.1 (2015-01-30)
------------------
* Update website and authors
* Add transform to transfer IMU data to base_link frame
* Make ROBOT_NETWORK optional
* Switch to robot_upstart python API
* Switch to debhelper install method for udeb rules
* Switch to env-hook for file storage
* Switch to new calibration method for um6; switch to imu_filter_magwick
* Contributors: Paul Bovbel

0.1.0 (2015-01-13)
------------------
* Port to robot_localization, gyro only pending um6 fixes
* changed the launch file to match parameter namespace changes in the imu_compass node
* ported kingfisher compass calibration to husky
* Added Microstrain device condition - Looks for an attached Microstrain device and installs the necessary launch files from the microstrain_config directory.
* Update sick.launch - Fixed binary name
* Change default IP for LIDAR to 192.168.1.14
* Add launcher for sick LIDAR.
* Added Microstrain launch file and udev rule.
* Contributors: Jeff Schmidt, Mike Purvis, Paul Bovbel, Prasenjit Mukherjee

0.0.6 (2013-10-12)
------------------
* Restore leading slash in checking the joystick path.
  This was removed by mistake in an earlier commit.

0.0.5 (2013-10-05)
------------------
* Acknowledge the ROBOT_SETUP env variable in the install script.

0.0.4 (2013-10-03)
------------------
* Remove the other launchfile check until we get a chance to fix the config location issue.
* adding installation of ekf yaml file to install script
* better parameters for husky compass calibration based on standard husky configurations
* combining both ekf launchers into one and relying on a config file to to pick whether we want an outdoor or indoor ekf to start
* allowing the user to scale the gps data if desired
* adding parameter to lock the altitude at 0
* set invalid covariance value for enu to really high, instead of -1

0.0.3 (2013-10-01)
------------------
* Add sicktoolbox_wrapper in advance of a config for standard LIDARs.
* Parameterize from environment variables the IMU and GPS ports, and network interface to launch from.

0.0.2 (2013-09-23)
------------------
* Compass startup and inertial ekf
* adding magnetometer configuration file to husky_bringup
* added static transform to um6 launcher
* Set namespace to navsat, baud rate to 9600.
* Depend on robot_upstart.
* Add automatic launchfile checks.

0.0.1 (2013-09-13)
------------------
* Catkinize package.
* First cut of a new install script.
