=========
Changelog
=========

[unreleased]
============

ouster_ros(1)
-------------
* breaking change: update to ouster_client release 20230403
* EOL notice: ouster-ros driver will drop support for ``ROS melodic`` by May 2023.
* bugfix: Address an issue causing the driver to warn about missing non-legacy fields even they exist
  in the original metadata file.
* added a new launch file ``sensor_mtp.launch`` for multicast use case (experimental).
* added a technique to estimate the the value of the lidar scan timestamp when it is missing packets
  at the beginning
* add frame_id to image topics
* fixed a potential issue of time values within generated point clouds that could result in a value
  overflow
* added a new ``/ouster/metadata`` topic that is consumed by os_cloud and os_image nodelets and
  save it to the bag file on record
* make specifying metadata file optional during record and replay modes as of package version 8.1
* added a no-bond option to the ``sensor.launch`` file
* reduce the publish rate of imu tf transforms

ouster_ros(2)
-------------
* MVP ouster-ros targeting ros2 distros (beta release)
* introduced a ``reset`` service to the ``os_sensor`` node

ouster_client
-------------
* added a new method ``mtp_init_client`` to init the client with multicast support (experimental).
* the class ``SensorHttp``  which provides easy access to REST APIs of the sensor has been made public
  under the ``ouster::sensor::util`` namespace.
* breaking change: get_metadata defaults to outputting non-legacy metadata
* add debug five_word profile which will be removed later
* breaking change: remove deprecations on LidarScan


[20230114]
==========

ouster_ros
----------
* breaking change: renamed ``ouster_ros/ros.h`` to ``ouster_ros/os_ros.h`` and
  ``ouster_ros/point.h`` to ``ouster_ros/os_point.h``.
* breaking change: change the type of the ring field within ``ouster::Point`` from ``uint8_t`` to
  ``uint16_t``
* correct LICENSE file installation path.
* update code files copyrights period.
* bug fix: ros driver doesn't use correct udp_dest given by user during launch
* update published TF transforms time with senosr or ros time based on the
  active timestamp mode.
* validate lidar and imu port values. warn users when assigning random port numbers.
* switch to using the cartesianT method when populating pcl point cloud for performance and reduced
  cpu utilization
* reduce dynamic memory allocation within the driver for performance and driver stability
* add ``pcl_ros`` as a dependency to ``package.xml``

ouster_client
--------------
* breaking change: signal multiplier type changed to double to support new FW values of signal
  multiplier.
* breaking change: make_xyz_lut takes mat4d beam_to_lidar_transform instead of
  lidar_origin_to_beam_origin_mm double to accomodate new FWs. Old reference Python implementation
  was kept, but new reference was also added.
* address an issue that could cause the processed frame being dropped in favor or the previous
  frame when the frame_id wraps-around.
* added a new flag ``CONFIG_FORCE_REINIT`` for ``set_config()`` method, to force the sensor to reinit
  even when config params have not changed.
* breaking change: drop defaults parameters from the shortform ``init_client()`` method.
* added a new method ``init_logger()`` to provide control over the logs emitted by ``ouster_client``.
* add parsing for new FW 3.0 thermal features shot_limiting and thermal_shutdown statuses and countdowns
* add frame_status to LidarScan
* introduced a new method ``cartesianT()`` which speeds up the computation of point projecion from range
  image, the method also can process the cartesian product with single float precision. A new unit test
  ``cartesian_test`` which shows achieved speed up gains by the number of valid returns in lidar scan.
* added ``RAW_HEADERS`` ChanField to LidarScan for packing headers and footer (alpha version, may be
  changed/removed without notice in the future)


[20221004]
==========

ouster_ros
----------
* Moved ouster-ros into separate repo
* Refresh the docker file

ouster_sdk
----------
* Removed ouster_ros
