# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;message_runtime;std_msgs;sensor_msgs;geometry_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-louster_ros".split(';') if "-louster_ros" != "" else []
PROJECT_NAME = "ouster_ros"
PROJECT_SPACE_DIR = "/home/kaijun/Documents/husky_autonomous/catkin_ws/install"
PROJECT_VERSION = "0.8.3"
