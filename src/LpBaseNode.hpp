// Copyright (C) 2021 LP-Research Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LP_BASE_NODE_HPP_
#define LP_BASE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cstring>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <optional>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>

#include "LpSlamTypes.h"

/*
nice explainer:
https://www.stereolabs.com/docs/ros2/video/
*/

namespace lpslam_components
{
class LpBaseNode : public rclcpp::Node
{
public:
    explicit LpBaseNode(const rclcpp::NodeOptions & options);

private:
    bool setParameters();
    void initTransforms();

public:
    // Sends map->odom transform
    void lpslam_OnReconstructionCallback(LpSlamGlobalStateInTime const &state_in_time);
    // Gets transform state between from_frame->to_frame
    LpSlamRequestNavTransformation lpslam_RequestNavTransformationCallback(LpSlamROSTimestamp ros_time,
        LpSlamNavDataFrame from_frame,
        LpSlamNavDataFrame to_frame);
    // Tries to get camera pose state in odom and map frames
    LpSlamRequestNavDataResult lpslam_RequestNavDataCallback(LpSlamROSTimestamp for_ros_time,
        LpSlamGlobalStateInTime * odometry,
        LpSlamGlobalStateInTime * map );

private:
    void computeAndPublishTransform(LpSlamGlobalStateInTime const & state_in_time);

protected:
    // Config makers
    bool make_openvslam_config(const sensor_msgs::msg::CameraInfo::SharedPtr right_msg, const sensor_msgs::msg::CameraInfo::SharedPtr left_msg);
    bool get_camera_color_order(YAML::Node & configNode);

    // ROS<->LP converters
    LpSlamGlobalStateInTime transformToLpSlamGlobalState(geometry_msgs::msg::TransformStamped const& tf) const;
    rclcpp::Time lpSlamToRosTime( LpSlamROSTimestamp const& ts ) const;
    LpSlamROSTimestamp rosTimeToLpSlam( rclcpp::Time const& ts ) const;

    // TF2 handlers
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;

    // frame ids
    std::string m_map_frame_id;
    std::string m_camera_frame_id;
    std::string m_odom_frame_id;
    std::string m_laserscan_frame_id;
    std::string m_base_frame_id;

    // Last map->odom transform
    std::optional<geometry_msgs::msg::TransformStamped> m_lastTransform;

    // configuration variables
    bool m_useOdometry;
    bool m_consumeLaser;
    bool m_isStereoCamera;
    // means that both stereo images are in one physical image
    // transferred via the ROS topic left_image_raw
    bool m_isCombinedStereoImage;
    std::string m_qosReliability;
    std::string m_laserscanTopic;
    std::string m_leftImageTopic;
    std::string m_rightImageTopic;
    bool m_useRosCameraInfo;
    std::string m_rightCameraInfoTopic;
    std::string m_leftCameraInfoTopic;
    double m_cameraFps;
    std::string m_pointcloudTopic;
    int m_pointcloudRate;
    std::string m_mapName;
    int m_mapRate;
    tf2::Duration m_transform_tolerance;

    // VSLAM method selection (currently OpenVSLAM supported)
    std::string m_vSlamMethod;

    // OpenVSLAM parameters
    int m_openVSlam_maxNumKeypoints;
    int m_openVSlam_iniMaxNumKeypoints;
    double m_openVSlam_scaleFactor;
    int m_openVSlam_numLevels;
    int m_openVSlam_iniFastThreshold;
    int m_openVSlam_minFastThreshold;
    double m_openVSlam_depthThreshold;
    double m_openVSlam_depthmapFactor;
    double m_openVSlam_mappingBaselineDistThr;
    double m_openVSlam_pangolinViewerFps;

    // Configuration file for OpenVSLAM
    std::string m_openVSlamYaml;
    // whether the camera config was read
    bool m_cameraConfigured;

private:
    // Indicator that SLAM has been started and one can use its outputs
    // (such as Map and PointClounds)
    bool m_slamStarted;
    std::mutex m_slamStartedMutex;

    // Camera color encoding OpenVSLAM parameter
    std::string m_openVSlam_cameraEncoding;
    std::mutex m_openVSlam_cameraEncodingMutex;

protected:
    // Thread-safe methods
    inline void setSlamStarted(const bool slam_started)
    {
        std::lock_guard<std::mutex> lock(m_slamStartedMutex);
        m_slamStarted = slam_started;
    }

    inline bool isSlamStarted()
    {
        std::lock_guard<std::mutex> lock(m_slamStartedMutex);
        return m_slamStarted;
    }

    inline void setCameraEncoding(const std::string & camera_encoding)
    {
        std::lock_guard<std::mutex> lock(m_openVSlam_cameraEncodingMutex);
        m_openVSlam_cameraEncoding = camera_encoding;
    }

    inline std::string getCameraEncoding()
    {
        std::lock_guard<std::mutex> lock(m_openVSlam_cameraEncodingMutex);
        return m_openVSlam_cameraEncoding;
    }
};

#endif  // LP_BASE_NODE_HPP_
   
} // namespace lpslam_components

