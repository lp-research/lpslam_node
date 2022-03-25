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

#ifndef OPENVSLAM_NODE_HPP_
#define OPENVSLAM_NODE_HPP_

#include "LpBaseNode.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <openvslam/system.h>
#include <openvslam/data/laser2d.h>
#include <openvslam/data/navigation_state.h>
#include <openvslam/data/occupancy_map_info.h>
#include <openvslam/data/landmark.h>
#include <openvslam/tracker_state.h>
#include <openvslam/publish/frame_state.h>
#include <openvslam/publish/map_publisher.h>

#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#endif  // USE_PANGOLIN_VIEWER

inline bool does_exist(std::string const& fname)
{
    std::ifstream infile(fname);
    return infile.good();
}

struct FeaturePosition {
  float x;
  // +y right
  float y;
  // +z forward
  float z;
};

namespace lpslam_components
{
   class OpenVSLAMNode : public LpBaseNode
{
public:
    OpenVSLAMNode(const rclcpp::NodeOptions & options);
    ~OpenVSLAMNode();

private:
#ifdef USE_PANGOLIN_VIEWER
    void viewerExecute();
#endif  // USE_PANGOLIN_VIEWER

    bool setParameters();
    void setSubscribers();
    void setPublishers();
    void setTimers();
    void resetTimers();

    void startSlam();

    void publishOccMap();
    void publishPointCloud();

    // callbacks
    void image_callback_stereo(
        const sensor_msgs::msg::Image::SharedPtr left,
        const sensor_msgs::msg::Image::SharedPtr right);
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

public:
    void stopSlam();

private:
    // publishers
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointcloudPublisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> m_occGridPublisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::MapMetaData>> m_mapMetaDataPublisher;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laserScanSubsription;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> m_leftImageSubscription;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> m_rightImageSubscription;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>
            m_synchronizer;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cameraInfoSubscription;

    // Timers
    std::shared_ptr<rclcpp::TimerBase> m_pointcloud_timer;
    std::shared_ptr<rclcpp::TimerBase> m_occmap_timer;

    // Last laserscan data
    openvslam::data::laser2d m_laser2D;
    builtin_interfaces::msg::Time m_laserTS;
    std::mutex m_laser2DMutex;

    // configuration variables
    std::string m_slamMode;
    bool m_mappingDuringLocalization;
    double m_maxLaserAge;

    // OpenVSLAM parameters
    std::string m_openVSlam_vocabFile;
    std::string m_openVSlam_mapDatabaseFile;

    // the godly SlamManager
    std::unique_ptr<openvslam::system> m_openVSlam;
    std::shared_ptr<openvslam::config> m_openVSlamCfg;
#ifdef USE_PANGOLIN_VIEWER
    std::unique_ptr<pangolin_viewer::viewer> m_viewer;
    std::shared_ptr<std::thread> m_viewerThread;
#endif  // USE_PANGOLIN_VIEWER
};

#endif  // OPENVSLAM_NODE_HPP_
 
} // namespace lpslam_components