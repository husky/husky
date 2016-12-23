^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
