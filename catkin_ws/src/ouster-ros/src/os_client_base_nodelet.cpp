/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_nodelet.cpp
 * @brief implementation of OusterClientBase interface
 */

#include "ouster_ros/os_client_base_nodelet.h"

#include <ouster/impl/build.h>
#include <std_msgs/String.h>

#include "ouster_ros/GetMetadata.h"

namespace sensor = ouster::sensor;
using ouster_ros::GetMetadata;

namespace nodelets_os {

void OusterClientBase::create_get_metadata_service(ros::NodeHandle& nh) {
    get_metadata_srv =
        nh.advertiseService<GetMetadata::Request, GetMetadata::Response>(
            "get_metadata",
            [this](GetMetadata::Request&, GetMetadata::Response& res) {
                res.metadata = cached_metadata;
                return cached_metadata.size() > 0;
            });

    NODELET_INFO("get_metadata service created");
}

void OusterClientBase::create_metadata_publisher(ros::NodeHandle& nh) {
    metadata_pub = nh.advertise<std_msgs::String>("metadata", 1, true);
}

void OusterClientBase::publish_metadata() {
    std_msgs::String metadata_msg;
    metadata_msg.data = cached_metadata;
    metadata_pub.publish(metadata_msg);
}

void OusterClientBase::display_lidar_info(const sensor::sensor_info& info) {
    auto lidar_profile = info.format.udp_profile_lidar;
    NODELET_INFO_STREAM(
        "ouster client version: "
        << ouster::SDK_VERSION_FULL << "\n"
        << "product: " << info.prod_line << ", sn: " << info.sn
        << ", firmware rev: " << info.fw_rev << "\n"
        << "lidar mode: " << sensor::to_string(info.mode) << ", "
        << "lidar udp profile: " << sensor::to_string(lidar_profile));
}

}  // namespace nodelets_os
