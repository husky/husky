/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_nodelet.h
 * @brief Base class for ouster_ros sensor and replay nodelets
 *
 */

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <ouster/types.h>

namespace nodelets_os {

class OusterClientBase : public nodelet::Nodelet {
   protected:
    bool is_arg_set(const std::string& arg) const {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void create_get_metadata_service(ros::NodeHandle& nh);

    void create_metadata_publisher(ros::NodeHandle& nh);

    void publish_metadata();

    void display_lidar_info(const ouster::sensor::sensor_info& info);

   protected:
    ouster::sensor::sensor_info info;
    ros::ServiceServer get_metadata_srv;
    std::string cached_metadata;
    ros::Publisher metadata_pub;
};

}  // namespace nodelets_os