# Official ROS1/ROS2 drivers for Ouster sensors

[ROS1 (melodic/noetic)](https://github.com/ouster-lidar/ouster-ros/tree/master) |
[ROS2 (rolling/humble)](https://github.com/ouster-lidar/ouster-ros/tree/ros2) |
[ROS2 (foxy)](https://github.com/ouster-lidar/ouster-ros/tree/ros2-foxy)

<p style="float: right;"><img width="20%" src="docs/images/logo.png" /></p>

| ROS Version | Build Status (Linux) |
|:-----------:|:------:|
| ROS1 (melodic/noetic) | [![melodic/noetic](https://github.com/ouster-lidar/ouster-ros/actions/workflows/docker-image.yml/badge.svg?branch=master)](https://github.com/ouster-lidar/ouster-ros/actions/workflows/docker-image.yml)
| ROS2 (rolling/humble) | [![rolling/humble](https://github.com/ouster-lidar/ouster-ros/actions/workflows/docker-image.yml/badge.svg?branch=ros2)](https://github.com/ouster-lidar/ouster-ros/actions/workflows/docker-image.yml)
| ROS2 (foxy) | [![foxy](https://github.com/ouster-lidar/ouster-ros/actions/workflows/docker-image.yml/badge.svg?branch=ros2-foxy)](https://github.com/ouster-lidar/ouster-ros/actions/workflows/docker-image.yml)

- [Overview](#overview)
- [Requirements](#requirements)
- [Getting Started](#getting-started)
- [Usage](#usage)
  - [Launching Nodes](#launching-nodes)
    - [Sensor Mode](#sensor-mode)
    - [Recording Mode](#recording-mode)
    - [Replay Mode](#replay-mode)
    - [Multicast Mode (experimental)](#multicast-mode-experimental)
  - [Invoking Services](#invoking-services)
    - [GetMetadata](#getmetadata)
    - [GetConfig](#getconfig)
    - [SetConfig (experimental)](#setconfig-experimental)
- [License](#license)


## Overview

This ROS package provide support for all Ouster sensors with FW v2.0 or later. Upon launch the driver
will configure and connect to the selected sensor device, once connected the driver will handle
incoming IMU and lidar packets, decode lidar frames and publish corresponding ROS messages on the
topics of `/ouster/imu` and `/ouster/points`. In the case the sensor supports dual return and it was
configured to use this capability, then another topic will published named `/ouster/points2` which
corresponds to the second point cloud.

## Requirements
This package only supports **Melodic** and **Noetic** ROS distros. Please refer to ROS online
documentation on how to setup ros on your machine before proceeding with the remainder of this guide.

In addition to the base ROS installation, the following ROS packages are required:
```bash
sudo apt install -y                     \
    ros-$ROS_DISTRO-pcl-ros             \
    ros-$ROS_DISTRO-rviz
```
where `$ROS-DISTRO` is either ``melodic`` or ``noetic``.

> **Note**  
> Installing `ros-$ROS_DISTRO-rviz` package is optional in case you didn't need to visualize the
> point cloud using rviz but remember to always set `viz` launch arg to `false`.
  

Additional dependenices:
```bash
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake
```
> **Note**  
> You may choose a different ssl backend for the curl library such as `libcurl4-gnutls-dev` or `libcurl4-nss-dev`

## Getting Started
To build the driver using ROS you need to clone the project into the `src` folder of a catkin workspace
as shown below:

```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
```

Next to compile the driver you need to source the ROS environemt into the active termainl:
```bash
source /opt/ros/<ros-distro>/setup.bash # replace ros-distro with 'melodic' or 'noetic'
```

Finally, invoke `catkin_make` command from within the catkin workspace as shown below:
```bash
cd catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Specifying `Release` as the build type is important to have a reasonable performance of the driver.


## Usage
### Launching Nodes
The package supports three modes of interaction, you can connect to a _live senosr_, _replay_ a
recorded bag or _record_ a new bag file using the corresponding launch files. Recently, we have
added a new mode that supports multicast. The commands are listed below:

#### Sensor Mode
```bash
roslaunch ouster_ros sensor.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>          # optional
```

#### Recording Mode
> Note
> As of package version 8.1, specifiying metadata file is optional since the introduction of the
> metadata topic
```bash
roslaunch ouster_ros record.launch      \
    sensor_hostname:=<sensor host name> \
    bag_file:=<optional bag file name>
    metadata:=<json file name>          # optional
```
#### Replay Mode
> Note
> As of package version 8.1, specifiying metadata file is optional if the bag file being replayed
> already contains the metadata topic

```bash
roslaunch ouster_ros replay.launch      \
    bag_file:=<path to rosbag file>     \
    metadata:=<json file name>          # optional if bag file has /metadata topic
```

#### Multicast Mode (experimental)
The multicast launch mode supports configuring the sensor to broadcast lidar packets from the same
sensor (live) to multiple active clients. You initiate this mode by using `sensor_mtp.launch` file
to start the node. You will need to specify a valid multicast group for the **udp_dest** argument
which the sensor is going to broadcast data to it. You will also need to set **mtp_main** argument
to **true**, this is need to configure the sensor with the specified **udp_dest** and any other
sensor settings. You can control on which ip (IP4 only) you wish to receive the data on this machine
from the multicast group using the **mtp_dest** argument
follows:
```bash
roslaunch ouster_ros sensor_mtp.launch      \
    sensor_hostname:=<sensor host name>     \
    udp_dest:=<multicast group ip (ipv4)>   \
    mtp_main:=true                          \
    mtp_dest:=<client ip to receive data>   # mtp_dest is optional
```
Using a different machine that belongs to the same netwok subnet, you can start another instance of
the client to start receiving sensor messages through the multicast group as shown below (note that
**mtp_main** is set to **false**):
```bash
roslaunch ouster_ros sensor_mtp.launch      \
    sensor_hostname:=<sensor host name>     \
    udp_dest:=<multicast group ip (ipv4)>   \
    mtp_main:=false                         \
    mtp_dest:=<client ip to receive data>   # mtp_dest is optional
```

> **Note:** 
> In both cases the **mtp_dest** is optional and if left unset the client will utilize the first
available interface.

### Invoking Services
To execute any of the following service, first you need to open a new terminal
and source the castkin workspace again by running the command:
`source catkin_ws/devel/setup.bash` 
#### GetMetadata
To get metadata while connected to a live sensor or during a replay session invoke
the following command:
```bash
rosservice call /ouster/get_metadata
```

#### GetConfig
To get the current config of a live sensor, invoke the command:
```bash
rosservice call /ouster/get_config
```

#### SetConfig (experimental)
To change config via a file while connected to a live sensor, invoke the command:
```bash
rosservice call /ouster/set_config "config_file: '<path to sensor config>'"
```

> **Note**
> Changing settings is not yet fully support during a reset operation (more on this)
  

For further detailed instructions refer to the [main guide](./docs/index.rst)


## License
[License File](./LICENSE)
