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

#ifndef LP_SLAM_NODE_HPP_
#define LP_SLAM_NODE_HPP_

#include "LpBaseNode.hpp"
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <lpslam_interfaces/msg/lp_slam_status.hpp>
#include "LpSlamManager.h"

void outside_lpslam_OnReconstructionCallback(LpSlamGlobalStateInTime const &state_in_time, void *lpslam_node);

LpSlamRequestNavDataResult outside_lpslam_RequestNavDataCallback(LpSlamROSTimestamp for_ros_time,
    LpSlamGlobalStateInTime * odometry,
    LpSlamGlobalStateInTime * map,
    void * lpslam_node);

LpSlamRequestNavTransformation outside_lpslam_RequestNavTransformationCallback(LpSlamROSTimestamp ros_time,
    LpSlamNavDataFrame from_frame,
    LpSlamNavDataFrame to_frame,
    void * lpslam_node);

namespace lpslam_components
{
class LpSlamNode : public LpBaseNode
{
public:
    LpSlamNode(const rclcpp::NodeOptions & options);
    ~LpSlamNode();

private:
    bool setParameters();
    void setSubscribers();
    void setPublishers();
    void setTimers();
    void resetTimers();

    void startSlam();

    void publishOccMap();
    void publishPointCloud();

    bool check_and_dispatch(const sensor_msgs::msg::Image::SharedPtr this_msg,
                            std::vector<uint8_t> &otherImageBuffer,
                            std::optional<builtin_interfaces::msg::Time> &otherTimestamp,
                            std::mutex &otherMutex,
                            bool isLeft);

    // callbacks
    void image_callback_right(const sensor_msgs::msg::Image::SharedPtr msg);
    void image_callback_left(const sensor_msgs::msg::Image::SharedPtr msg);
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr left_msg, const sensor_msgs::msg::CameraInfo::SharedPtr right_msg);

    // Config makers
    bool make_lpslam_config();

    void stopSlam();

    // publishers
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointcloudPublisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> m_occGridPublisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::MapMetaData>> m_mapMetaDataPublisher;
    std::shared_ptr<rclcpp::Publisher<lpslam_interfaces::msg::LpSlamStatus>> m_slamStatusPublisher;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laserScanSubsription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_leftImageSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_rightImageSubscription;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> m_leftCameraInfoSubscription;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> m_rightCameraInfoSubscription;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>>
            m_camInfoSynchronizer;

    // Timers
    std::shared_ptr<rclcpp::TimerBase> m_pointcloud_timer;
    std::shared_ptr<rclcpp::TimerBase> m_occmap_timer;
    std::shared_ptr<rclcpp::TimerBase> m_slamstatus_timer;

    // buffering of incoming images to compile one stereo image
    std::vector<uint8_t> m_rightImageBuffer;
    std::optional<builtin_interfaces::msg::Time> m_rightImageTimestamp;
    std::mutex m_rightImageMutex;

    std::vector<uint8_t> m_leftImageBuffer;
    std::optional<builtin_interfaces::msg::Time> m_leftImageTimestamp;
    std::mutex m_leftImageMutex;

    // configuration variables
    std::string m_configFile;
    std::string m_lpSlamLogFile;
    bool m_writeLpSlamLog;
    double m_timespanMatch;
    std::string m_lpslamStatusTopic;

    // the godly SlamManager
    LpSlamManager m_slam;
    // LpSlam input JSON file
    std::string m_lpSlamJson;
};

#endif  // LP_SLAM_NODE_HPP_

    
} // namespace lpslam_components