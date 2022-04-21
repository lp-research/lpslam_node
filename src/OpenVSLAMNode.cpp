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

#include "OpenVSLAMNode.hpp"

namespace lpslam_components
{
OpenVSLAMNode::OpenVSLAMNode(const rclcpp::NodeOptions & options) : LpBaseNode(options)
{
    RCLCPP_INFO(get_logger(), "starting OpenVSlam node");

    if (!setParameters()) {
        return;
    }

    setSubscribers();
    setPublishers();

    if (m_cameraInfoTopic.empty()) {
        // Camera calibration files won't be read from topic.
        // Starting SLAM manually.
        startSlam();
    }

    setTimers();

    if (!setCameraPlacement()) {
        RCLCPP_INFO(get_logger(), "No camera transform available, assuming it's mounted at z = 0.0 for now");
        m_cameraZ = 0.0;
    }
}

OpenVSLAMNode::~OpenVSLAMNode()
{
    RCLCPP_INFO(get_logger(), "shutting down OpenVSlam");
    resetTimers();
    stopSlam();
    RCLCPP_INFO(get_logger(), "shutting down OpenVSlam complete");
}

#ifdef USE_PANGOLIN_VIEWER
void OpenVSLAMNode::viewerExecute()
{
    if (m_viewer) {
        m_viewer->run();
    }
}
#endif  // USE_PANGOLIN_VIEWER

bool OpenVSLAMNode::setParameters()
{
    // configuration variables
    m_slamMode = this->declare_parameter<std::string>("slam_mode", "slam");
    m_mappingDuringLocalization = this->declare_parameter<bool>(
        "mapping_during_localization", true);
    m_maxLaserAge = this->declare_parameter<double>("max_laser_age", 0.5);
    m_laserTS.sec = 0;
    m_laserTS.nanosec = 0;

    // OpenVSLAM parameters
    m_openVSlam_vocabFile = this->declare_parameter<std::string>(m_vSlamMethod + "." + "vocab_file", "");
    if (!m_openVSlam_vocabFile.size()) {
        RCLCPP_ERROR(get_logger(), "%s vocab_file is not specified", m_vSlamMethod.c_str());
        return false;
    }
    m_openVSlam_mapDatabaseFile =
        this->declare_parameter<std::string>(m_vSlamMethod + "." + "map_database", "map.db");

    return true;
}

void OpenVSLAMNode::setSubscribers()
{
    // allow for relaxed QoS for laser scan in order to match
    // the topic settings
    auto laser_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    if (m_qosReliability == "best_effort") {
        laser_qos.best_effort();
    } else {
        // reliable
        laser_qos.reliable();
    }

    if (m_consumeLaser) {
        m_laserScanSubsription = this->create_subscription<sensor_msgs::msg::LaserScan>(
            m_laserscanTopic, laser_qos,
            std::bind(&OpenVSLAMNode::laserscan_callback, this, std::placeholders::_1));
    }

    if (m_isStereoCamera && !m_isCombinedStereoImage)
    {
        // allow for relaxed QoS for image in order to match
        // the topic settings
        rmw_qos_profile_t video_qos = rmw_qos_profile_sensor_data;
        video_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        video_qos.depth = 10;
        if (m_qosReliability == "best_effort") {
            video_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        } else {
            // reliable
            video_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        }

        m_leftImageSubscription =
            std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, m_leftImageTopic, video_qos);
        m_rightImageSubscription =
            std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, m_rightImageTopic, video_qos);
        m_synchronizer = std::make_shared<
            message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(
                *m_leftImageSubscription, *m_rightImageSubscription, 10);
        m_synchronizer->registerCallback(&OpenVSLAMNode::image_callback_stereo, this);
    }

    if (!m_cameraInfoTopic.empty())
    {
        // allow for relaxed QoS for image in order to match
        // the topic settings
        auto video_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        video_qos.keep_last(10);
        if (m_qosReliability == "best_effort") {
            video_qos.best_effort();
        } else {
            // reliable
            video_qos.reliable();
        }

        m_cameraInfoSubscription = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            m_cameraInfoTopic, video_qos,
            std::bind(&OpenVSLAMNode::camera_info_callback, this, std::placeholders::_1));
    }
}

void OpenVSLAMNode::setPublishers()
{
    m_pointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        m_pointcloudTopic, 1);

    m_occGridPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(m_mapName,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    m_mapMetaDataPublisher = this->create_publisher<nav_msgs::msg::MapMetaData>(m_mapName + "_metadata",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void OpenVSLAMNode::setTimers()
{
    // start publishing point cloud
    auto ros_clock = rclcpp::Clock::make_shared();
    m_pointcloud_timer = rclcpp::create_timer(this, ros_clock, rclcpp::Duration(m_pointcloudRate, 0),
                                              [this]() {
                                                  this->publishPointCloud();
                                              });
    m_occmap_timer = rclcpp::create_timer(this, ros_clock, rclcpp::Duration(m_mapRate, 0),
                                              [this]() {
                                                  this->publishOccMap();
                                              });
}

void OpenVSLAMNode::resetTimers()
{
    m_pointcloud_timer.reset();
    m_occmap_timer.reset();
}

void OpenVSLAMNode::startSlam()
{
    if (m_openVSlamYaml.empty()) {
        RCLCPP_ERROR(get_logger(), "OpenVSLAM config yaml is not being set or prepared");
        return;
    }

    try {
        m_openVSlamCfg = std::make_shared<openvslam::config>(m_openVSlamYaml);
        m_openVSlamCfg->write_logfile_ = true;
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to load OpenVSLAM config file %s because %s",
            m_openVSlamYaml.c_str(), ex.what());
        return;
    }
    m_openVSlam = std::make_unique<openvslam::system>(m_openVSlamCfg, m_openVSlam_vocabFile);

    if (m_slamMode == "slam") {
        RCLCPP_INFO(get_logger(), "Starting SLAM in mapping mode");

        // Start SLAM in mapping mode
        m_openVSlam->startup(true);
    } else {
        RCLCPP_INFO(get_logger(), "Starting SLAM in localization mode");

        // Start SLAM in localization mode
        m_openVSlam->startup(false);

        // Local OpenVSLAM map database from file
        if (does_exist(m_openVSlam_mapDatabaseFile)) {
            RCLCPP_INFO(
                get_logger(),
                "Loading OpenVSLAM map database from %s",
                m_openVSlam_mapDatabaseFile.c_str());
            m_openVSlam->load_map_database(m_openVSlam_mapDatabaseFile);
        }

        // Enable or disable further mapping while working in localization mode
        if (m_mappingDuringLocalization) {
            RCLCPP_INFO(get_logger(), "Mapping is enabled in localization mode");
            m_openVSlam->enable_mapping_module();
        } else {
            RCLCPP_INFO(get_logger(), "Mapping is disabled in localization mode");
            m_openVSlam->disable_mapping_module();
        }
    }

#ifdef USE_PANGOLIN_VIEWER
    m_viewer = std::make_unique<pangolin_viewer::viewer>(
        openvslam::util::yaml_optional_ref(m_openVSlamCfg->yaml_node_, "PangolinViewer"),
        m_openVSlam.get(), m_openVSlam->get_frame_publisher(), m_openVSlam->get_map_publisher());
    m_viewerThread = std::make_shared<std::thread>(&OpenVSLAMNode::viewerExecute, this);
#endif  // USE_PANGOLIN_VIEWER

    setSlamStarted(true);
    RCLCPP_INFO(get_logger(), "OpenVSLAM processing starting");
}

// according to https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/OccupancyGrid.msg
// http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html
void OpenVSLAMNode::publishOccMap()
{
    if (!isSlamStarted()) {
        return;
    }

    if (m_occGridPublisher->get_subscription_count() == 0) {
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Start publishing occupancy map");
    const unsigned long rawSizeNeeded =
        m_openVSlam->get_map_publisher()->occupancy_map_export_size_required();

    nav_msgs::msg::OccupancyGrid occGridMessage;
    occGridMessage.data.resize(rawSizeNeeded);

    const openvslam::data::occupancy_map_info mapInfo =
        m_openVSlam->get_map_publisher()->occupancy_map_export(
            occGridMessage.data.data(),
            rawSizeNeeded);
    const unsigned long exportCount = mapInfo.width * mapInfo.height;
    RCLCPP_DEBUG(get_logger(), "exported %lu occupancy map points", exportCount);

    occGridMessage.info.height = mapInfo.height;
    occGridMessage.info.width = mapInfo.width;
    occGridMessage.info.resolution = mapInfo.resolution;
    occGridMessage.info.origin.position.x = mapInfo.origin_x;
    occGridMessage.info.origin.position.y = mapInfo.origin_y;

    RCLCPP_DEBUG(get_logger(), "occupancy map origin (%f, %f)", occGridMessage.info.origin.position.x,
        occGridMessage.info.origin.position.y);

    //  account for higher placement of camera
    if(m_cameraZ == 0.0){
        // try to set camera placement again
        setCameraPlacement();
        RCLCPP_INFO(get_logger(), "Camera z = %f", m_cameraZ);
    }
    occGridMessage.info.origin.position.z = m_cameraZ;
    occGridMessage.header.frame_id = m_map_frame_id;
    occGridMessage.header.stamp = this->now();

    m_occGridPublisher->publish(std::move(occGridMessage));
    m_mapMetaDataPublisher->publish(std::move(occGridMessage.info));
    RCLCPP_DEBUG(get_logger(), "Finished publishing occupancy map");
}

void OpenVSLAMNode::publishPointCloud()
{
    if (!isSlamStarted()) {
        return;
    }

    if (m_pointcloudPublisher->get_subscription_count() == 0) {
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Starting to publish feature points");

    if (sizeof(FeaturePosition) != (sizeof(float) * 3))
    {
        RCLCPP_ERROR(get_logger(), "Assumption about FeaturePosition struct size not correct");
        return;
    }

    std::vector<FeaturePosition> featurePointBuffer;
    // limit the max points for now
    const size_t maxTotalFeatureSize = std::size_t(50000);

    // from mappingGetFeatures()
    auto lmd = [&featurePointBuffer, maxTotalFeatureSize]
        ( std::unordered_map<unsigned int, openvslam::data::landmark*> const& features ) -> void {
            size_t bufferSize = 0;
            // transform from LpSlam coords to ROS
            //transformation lpslam -> ROS is
            // z_ros = x_lpslam
            // x_ros = z_lpslam
            // y_ros = -y_lpslam
            Eigen::Matrix3f lpslam_to_ros;
            lpslam_to_ros << 0.0,  0.0,  1.0,
                             0.0, -1.0,  0.0,
                             1.0,  0.0,  0.0;

            for ( auto const& ft: features) {
                auto const worldPos = ft.second->get_pos_in_world();
                Eigen::Vector3f p_lpslam;//(-worldPos.y(), worldPos.x(), worldPos.z());
                p_lpslam << -worldPos.y(), worldPos.x(), worldPos.z();
                const Eigen::Vector3f p_transformed = lpslam_to_ros * p_lpslam;

                featurePointBuffer.push_back({p_transformed.x(), p_transformed.y(), p_transformed.z()});
                bufferSize++;
                if (bufferSize >= maxTotalFeatureSize) {
                    break;
                }
            }
        };

    std::function<void(std::unordered_map<unsigned int, openvslam::data::landmark*> const&)> funcLamda
         = lmd;

    m_openVSlam->get_map_publisher()->execute_on_landmarks(funcLamda);

    const size_t outputSize = featurePointBuffer.size();

    RCLCPP_DEBUG_STREAM(get_logger(), "Streaming out " << outputSize << " feature points");

    // from https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

    // Create a PointCloud2
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // Fill some internals of the PoinCloud2 like the header/width/height ...
    cloud_msg->header.frame_id = m_map_frame_id;
    cloud_msg->header.stamp = this->now();

    cloud_msg->height = 1;
    cloud_msg->width = 3;
    // Set the point fields to xyzrgb and resize the vector with the following command
    // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
    // the number of occurrences of the type in the PointField, the type of the PointField
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg.get());
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
    // You have to be aware that the following function does add extra padding for backward compatibility though
    // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
    // 2 is for the number of fields to add
    //modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // You can then reserve / resize as usual
    modifier.resize(outputSize);

    cloud_msg->is_bigendian = false;
    cloud_msg->is_dense = false;

    float* cloud_msg_ptr = (float*)(&cloud_msg->data[0]);
    std::memcpy(cloud_msg_ptr, (float*)featurePointBuffer.data(), outputSize * sizeof(FeaturePosition));

    m_pointcloudPublisher->publish(std::move(cloud_msg));
    RCLCPP_DEBUG(get_logger(), "Finished publishing feature points");
}

// Callbacks
void OpenVSLAMNode::image_callback_stereo(
    const sensor_msgs::msg::Image::SharedPtr left,
    const sensor_msgs::msg::Image::SharedPtr right)
{
    const std::string camera_encoding = getCameraEncoding();
    if (camera_encoding.empty()) {
        setCameraEncoding(left->encoding);
        // OpenVSLAM is not configured yet. No need to continue.
        return;
    }

    if (!m_useOdometry) {
        // Don't continue if there is no odometry data
        RCLCPP_WARN(get_logger(), "Odometry data was disabled, skipping frame");
        return;
    }

    auto leftcv = cv_bridge::toCvShare(left)->image;
    auto rightcv = cv_bridge::toCvShare(right)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    const double img_timestamp = left->header.stamp.sec + left->header.stamp.nanosec / 1e9;

    // Get last camera pose in odom and map frame (if any)
    LpSlamGlobalStateInTime last_odom_pose, last_map_pose;
    LpSlamRequestNavDataResult res = lpslam_RequestNavDataCallback(
        rosTimeToLpSlam(left->header.stamp),
        &last_odom_pose, &last_map_pose);

    // Convert camera pose in odom and map frames
    // from LpSlamGlobalStateInTime -> to openvslam::navigation_state
    openvslam::navigation_state cam_pose_odom, cam_pose_map;
    cam_pose_odom.velocity_valid = false;  // Currently not utilized by LP-OpenVSLAM
    cam_pose_map.velocity_valid = false;  // Currently not utilized by LP-OpenVSLAM
    if (res == LpSlamRequestNavDataResult_OdomAndMap ||
        res == LpSlamRequestNavDataResult_OdomOnly) {
        // Last camera state in odom frame is valid
        cam_pose_odom.valid = true;
        cam_pose_odom.cam_translation = openvslam::Vec3_t(
            last_odom_pose.state.position.y,
            -last_odom_pose.state.position.x,
            last_odom_pose.state.position.z);
        const Eigen::Quaternion<double> cam_rot_q = Eigen::Quaternion<double>(
            last_odom_pose.state.orientation.w,
            last_odom_pose.state.orientation.y,
            -last_odom_pose.state.orientation.x,
            last_odom_pose.state.orientation.z);
        cam_pose_odom.cam_rotation = cam_rot_q.normalized().toRotationMatrix();
    } else {
        // Last camera state in odom frame is invalid:
        // don't continue if there is no odometry data
        RCLCPP_ERROR(
            get_logger(),
            "Skipping camera frame for tracking because no odometry information available");
        return;
    }

    if (res == LpSlamRequestNavDataResult_OdomAndMap ||
        res == LpSlamRequestNavDataResult_MapOnly) {
        // Last camera state in map frame is valid
        cam_pose_map.valid = true;
        cam_pose_map.cam_translation = openvslam::Vec3_t(
            last_map_pose.state.position.y,
            -last_map_pose.state.position.x,
            last_map_pose.state.position.z);
        const Eigen::Quaternion<double> cam_rot_q = Eigen::Quaternion<double>(
            last_map_pose.state.orientation.w,
            last_map_pose.state.orientation.y,
            -last_map_pose.state.orientation.x,
            last_map_pose.state.orientation.z);
        cam_pose_map.cam_rotation = cam_rot_q.normalized().toRotationMatrix();
    } else {
        // Last camera state in map frame is invalid
        cam_pose_map.valid = false;
    }

    {
        std::lock_guard<std::mutex> lock(m_laser2DMutex);

        // Check whether laser scanner data is not outdated
        if (m_laser2D.is_valid()) {
            const int64_t dt_sec = static_cast<int64_t>(m_laserTS.sec) -
                static_cast<int64_t>(left->header.stamp.sec);
            const int64_t dt_nanosec = static_cast<int64_t>(m_laserTS.nanosec) -
                static_cast<int64_t>(left->header.stamp.nanosec);
            const double dt = dt_sec + dt_nanosec / 1e9;

            if (std::abs(dt) < m_maxLaserAge) {
                RCLCPP_INFO(get_logger(), "Selected laser data with dt %f to use", dt);
            } else {
                RCLCPP_ERROR(get_logger(), "Laser data with age %f is to old too use", dt);
                m_laser2D.ranges_.clear();
            }
        }

        // Track image
        m_openVSlam->feed_stereo_frame(
            leftcv, rightcv, img_timestamp, cv::Mat{},
            cam_pose_odom, cam_pose_map, m_laser2D);

        // Get camera pose after VSLAM
        const Eigen::Matrix4d cam_pose_cw = m_openVSlam->get_map_publisher()->get_current_cam_pose();

        // Get tracking state
        LpSlamGlobalStateInTime res;
        const openvslam::publish::frame_state frame_state = m_openVSlam->get_frame_state();
        if (frame_state.tracker_state ==
            openvslam::tracker_state_t::Tracking)
        {
            // Fill the state
            //cam_center = -rot_cw.transpose() * trans_cw;
            Eigen::Matrix3d rot_cw = cam_pose_cw.block<3, 3>(0, 0);
            Eigen::Quaterniond q_rot_cw(rot_cw);
            Eigen::Vector3d trans_cw = cam_pose_cw.block<3, 1>(0, 3);
            Eigen::Vector3d cam_center = -rot_cw.transpose() * trans_cw;
            // Convert pose from optical to LP coordinate system
            res.state.position.x = -cam_center.y();
            res.state.position.y = cam_center.x();
            res.state.position.z = cam_center.z();
            res.state.orientation.w = q_rot_cw.w();
            res.state.orientation.x = -q_rot_cw.y();
            res.state.orientation.y = q_rot_cw.x();
            res.state.orientation.z = q_rot_cw.z();
            res.state.valid = true;

            // Fill the time
            res.ros_timestamp.seconds = left->header.stamp.sec;
            res.ros_timestamp.nanoseconds = left->header.stamp.nanosec;
            res.has_ros_timestamp = true;
        } else {
            std::string state_string;
            switch (frame_state.tracker_state) {
                case openvslam::tracker_state_t::NotInitialized:
                    state_string = "Not Initialized";
                    break;
                case openvslam::tracker_state_t::Initializing:
                    state_string = "Initializing";
                    break;
                case openvslam::tracker_state_t::Lost:
                    state_string = "Track Lost";
                    break;
                default:
                    state_string = "Unknown";
                    break;
            }
            RCLCPP_WARN(
                get_logger(),
                "OpenVSLAM tracker is in \"%s\" state",
                state_string.c_str());

            // Send invalid reconstructon in order to keep TF being updated
            // (to latest valid map->odom state)
            res.state.valid = false;
        }

        // Publish map->odom transform
        lpslam_OnReconstructionCallback(res);
    }
}

void OpenVSLAMNode::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // This can be used to transform laser scan into camera frame
    // http://docs.ros.org/en/latest/api/tf2_ros/html/c++/classtf2__ros_1_1BufferInterface.html#a12d0bda7d7533f199e94dae4bb46ba7a
    // Use this method to lookup the transform of the laser data at time t_l to the camera pose at t_c

    RCLCPP_DEBUG(get_logger(), "Received %lu laser scans with min %f max %f angle start %f angle end %f",
        msg->ranges.size(), msg->range_min, msg->range_max, msg->angle_min, msg->angle_max);

    {
        std::lock_guard<std::mutex> lock(m_laser2DMutex);

        // Copying LaserScan data to laser2d
        const size_t ranges_num = msg->ranges.size();
        m_laser2D.ranges_.resize(ranges_num);
        std::memcpy(m_laser2D.ranges_.data(), msg->ranges.data(), sizeof(float) * ranges_num);

        // Setting other laser2d parameters
        m_laser2D.angle_min_ = msg->angle_min;
        m_laser2D.angle_max_ = msg->angle_max;
        m_laser2D.angle_increment_ = msg->angle_increment;
        m_laser2D.range_min_ = msg->range_min;
        m_laser2D.range_max_ = msg->range_max;

        // Obtaining m_laser2D.trans_lc_ and m_laser2D.rot_lc_
        if (m_laser2D.is_valid()) {
            // lookup transformation for laser data to camera!
            const LpSlamGlobalState lp_laser_to_camera =
                lpslam_RequestNavTransformationCallback(
                    rosTimeToLpSlam(msg->header.stamp),
                    LpSlamNavDataFrame_Laser, LpSlamNavDataFrame_Camera);

            // Get relative movement between camera and laser at that time !
            if (lp_laser_to_camera.valid) {
                // Transform from laser -> to camera frame (or laser origin in camera's frame)
                // Converting from LP to optical coordinate system.
                m_laser2D.trans_lc_ = openvslam::Vec3_t(
                    lp_laser_to_camera.position.y,
                    -lp_laser_to_camera.position.x,
                    lp_laser_to_camera.position.z);
                const Eigen::Quaternion<double> vslam_r_lc = Eigen::Quaternion<double>(
                    lp_laser_to_camera.orientation.w,
                    lp_laser_to_camera.orientation.y,
                    -lp_laser_to_camera.orientation.x,
                    lp_laser_to_camera.orientation.z);
                m_laser2D.rot_lc_ = vslam_r_lc.normalized().toRotationMatrix();
            }
        }

        m_laserTS = msg->header.stamp;
    }
    RCLCPP_DEBUG(get_logger(), "Laser data dispatched");
}

void OpenVSLAMNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (m_cameraConfigured)
    {
        return;
    }

    // Make OpenVSLAM config-file
    if (!make_openvslam_config(msg))
    {
        return;
    }

    // All configurations were prepared. Starting SLAM.
    startSlam();

    // Once camera was configured, no more actions needed here
    m_cameraConfigured = true;
}

void OpenVSLAMNode::stopSlam()
{
    // only save if we used the map and and mapping was enabled
    if (m_slamMode == "slam") {
        if (m_openVSlam_mapDatabaseFile.size()) {
            RCLCPP_INFO(
                get_logger(),
                "Saving mapping database to %s", m_openVSlam_mapDatabaseFile.c_str());
            m_openVSlam->save_map_database(m_openVSlam_mapDatabaseFile);
        } else {
            RCLCPP_WARN(get_logger(), "Won't save mapping because file name is invalid");
        }
    }

#ifdef USE_PANGOLIN_VIEWER
    m_viewerThread.reset();
    m_viewer->request_terminate();
    m_viewer.reset(nullptr);
#endif  // USE_PANGOLIN_VIEWER

    m_openVSlam->shutdown();
    m_openVSlam.reset(nullptr);
}

    
} // namespace lpslam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lpslam_components::OpenVSLAMNode)
