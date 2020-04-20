^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2020-04-20)
------------------

0.3.5 (2019-12-11)
------------------
* Added envar for joy device.
* Contributors: Dave Niewinski, Tony Baltovski

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
