^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.13 (2022-11-25)
-------------------
* Fixed all scan topics to use front/scan.
* Contributors: Tony Baltovski

0.4.12 (2022-01-17)
-------------------

0.4.11 (2022-01-14)
-------------------

0.4.10 (2021-07-18)
-------------------

0.4.9 (2021-07-15)
------------------

0.4.8 (2021-04-01)
------------------

0.4.7 (2021-03-16)
------------------

0.4.6 (2021-03-09)
------------------

0.4.5 (2020-10-01)
------------------
* Expose the scan_topic argument in amcl_demo and gmapping_demo, make them use the HUSKY_LASER_TOPIC env var as their default
* Contributors: Chris Iverach-Brereton

0.4.4 (2020-08-13)
------------------
* Removed Paul Bovbel as maintainer.
* Contributors: Tony Baltovski

0.4.3 (2020-04-20)
------------------

0.4.2 (2019-12-11)
------------------

0.4.1 (2019-09-30)
------------------

0.4.0 (2019-08-01)
------------------
* Removed frontier_exploration temporarily for Melodic.
* Contributors: Tony Baltovski

0.3.4 (2019-08-01)
------------------

0.3.3 (2019-04-18)
------------------

0.3.2 (2019-03-25)
------------------

0.3.1 (2018-08-02)
------------------

0.3.0 (2018-04-11)
------------------
* Updated all package versions to 0.2.6.
* Fixed typo in URLs.
* Remove defunct email address
* Updated maintainers.
* Add interface definitions
* Contributors: Paul Bovbel, Tony Baltovski

0.2.7 (2015-12-31)
------------------
* Removed move_base topic remap so it publishes to just cmd_vel to avoid confusion.
* Remapping the move_base topic to be compatible with cpr autonomy core.
* Contributors: Peiyi Chen

0.2.6 (2015-07-08)
------------------

0.2.5 (2015-04-16)
------------------

0.2.4 (2015-04-13)
------------------

0.2.3 (2015-04-08)
------------------
* Increase inflation radius
* Contributors: Paul Bovbel

0.2.2 (2015-03-23)
------------------
* Fix package urls
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------

0.2.0 (2015-03-23)
------------------

0.1.2 (2015-02-17)
------------------

0.1.1 (2015-01-30)
------------------
* Update web and maintainers
* More meaningful name for mapless navigation demo
* Add missing dependencies
* Contributors: Paul Bovbel

0.1.0 (2015-01-15)
------------------
* Add exploration demo
* Indigo release refactor
* adding prebuilt maps for playpen, slightly more representative map then willowgarage world
* Contributors: Paul Bovbel, Prasenjit Mukherjee

0.0.6 (2013-10-05)
------------------
* Remove attempt to install now gone laser directory.

0.0.5 (2013-10-05)
------------------
* Depend on roslaunch for the check macro.

0.0.4 (2013-10-04)
------------------
* restructuring launch file locations, cleaning out redundant 'laser' folder
* adding dependencies to the package and launch file test in CMake

0.0.3 (2013-10-04)
------------------
* Install the right things.
* Remove urdf folder.

0.0.2 (2013-10-04)
------------------
* cleanup of odom_navigation file and adding tolerance parameters
* removed legacy outdoor navigation demo
* moving param files for odometry navigation around and changed the launchfile to reflect change
* first commit of move_base launch and configuration files in the odom frame
* first commit of catkinized version
