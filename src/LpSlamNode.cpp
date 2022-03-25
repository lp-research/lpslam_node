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

#include "LpSlamNode.hpp"
namespace lpslam_components
{

void outside_lpslam_OnReconstructionCallback(LpSlamGlobalStateInTime const &state_in_time, void *lpslam_node)
{
    static_cast<LpSlamNode *>(lpslam_node)->lpslam_OnReconstructionCallback(state_in_time);
}

LpSlamRequestNavDataResult outside_lpslam_RequestNavDataCallback(LpSlamROSTimestamp for_ros_time,
    LpSlamGlobalStateInTime * odometry,
    LpSlamGlobalStateInTime * map,
    void * lpslam_node)
{
    return static_cast<LpSlamNode *>(lpslam_node)->lpslam_RequestNavDataCallback(for_ros_time,
        odometry, map);
}

LpSlamRequestNavTransformation outside_lpslam_RequestNavTransformationCallback(LpSlamROSTimestamp ros_time,
    LpSlamNavDataFrame from_frame,
    LpSlamNavDataFrame to_frame,
    void * lpslam_node)
{
    return static_cast<LpSlamNode *>(lpslam_node)->lpslam_RequestNavTransformationCallback(ros_time,
        from_frame, to_frame);
}

   LpSlamNode::LpSlamNode(const rclcpp::NodeOptions & options) : LpBaseNode(options)
{
    RCLCPP_INFO(get_logger(), "starting LpSlam node");

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
}

LpSlamNode::~LpSlamNode()
{
    RCLCPP_INFO(get_logger(), "shutting down LpSlam");
    resetTimers();
    stopSlam();
    RCLCPP_INFO(get_logger(), "shutting down LpSlam complete");
}

bool LpSlamNode::setParameters()
{
    m_configFile = this->declare_parameter<std::string>("lpslam_config", "");
    m_lpSlamLogFile = this->declare_parameter<std::string>("lpslam_log_file", "lpslam.log");
    m_writeLpSlamLog = this->declare_parameter<bool>("write_lpslam_log", false);
    m_timespanMatch = this->declare_parameter<double>("timespan_match", 0.010); // 10 ms mach distance

    m_lpslamStatusTopic = this->declare_parameter<std::string>(
        "lpslam_status_topic", "lpslam_status");

    m_lpSlamJson = m_configFile;

    return true;
}

void LpSlamNode::setSubscribers()
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
            std::bind(&LpSlamNode::laserscan_callback, this, std::placeholders::_1));
    }

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

    m_leftImageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
        m_leftImageTopic, video_qos,
        std::bind(&LpSlamNode::image_callback_left, this, std::placeholders::_1));
    if (m_isStereoCamera && !m_isCombinedStereoImage)
    {
        m_rightImageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            m_rightImageTopic, video_qos,
            std::bind(&LpSlamNode::image_callback_right, this, std::placeholders::_1));
    }

    if (!m_cameraInfoTopic.empty())
    {
        m_cameraInfoSubscription = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            m_cameraInfoTopic, video_qos,
            std::bind(&LpSlamNode::camera_info_callback, this, std::placeholders::_1));
    }
}

void LpSlamNode::setPublishers()
{
    m_pointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        m_pointcloudTopic, 1);

    m_occGridPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(m_mapName,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    m_mapMetaDataPublisher = this->create_publisher<nav_msgs::msg::MapMetaData>(m_mapName + "_metadata",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    m_slamStatusPublisher = this->create_publisher<lpslam_interfaces::msg::LpSlamStatus>(
        m_lpslamStatusTopic,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void LpSlamNode::setTimers()
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
    m_slamstatus_timer = rclcpp::create_timer(this, ros_clock, rclcpp::Duration(1, 0),
                                              [this]() {
                                                  if (!isSlamStarted()) {
                                                      return;
                                                  }

                                                  const auto state = this->m_slam.getSlamStatus();

                                                  lpslam_interfaces::msg::LpSlamStatus ros_state;

                                                    ros_state.localization_status = 0;
                                                    if (state.localization == LpSlamLocalization_Off) {
                                                        ros_state.localization_status = 0;
                                                    } else if (state.localization == LpSlamLocalization_Initializing) {
                                                        ros_state.localization_status = 1;
                                                    } else if (state.localization == LpSlamLocalization_Tracking) {
                                                        ros_state.localization_status = 2;
                                                    } else if (state.localization == LpSlamLocalization_Lost) {
                                                        ros_state.localization_status = 3;
                                                    }

                                                  ros_state.feature_points = state.feature_points;
                                                  ros_state.key_frames = state.key_frames;
                                                  ros_state.frame_time = state.frame_time;
                                                  ros_state.fps = state.fps;

                                                  this->m_slamStatusPublisher->publish(std::move(ros_state));
                                              });
}

void LpSlamNode::resetTimers()
{
    m_pointcloud_timer.reset();
    m_occmap_timer.reset();
    m_slamstatus_timer.reset();
}

void LpSlamNode::startSlam()
{
    if (m_writeLpSlamLog)
    {
        m_slam.logToFile(m_lpSlamLogFile.c_str());
    }
    m_slam.setLogLevel(LpSlamLogLevel_Debug);
    RCLCPP_INFO_STREAM(get_logger(), "Loading LpSlam configuration from " << m_lpSlamJson);
    m_slam.readConfigurationFile(m_lpSlamJson.c_str());

    m_slam.addOnReconstructionCallback(&outside_lpslam_OnReconstructionCallback, this);
    m_slam.addRequestNavTransformation(&outside_lpslam_RequestNavTransformationCallback, this);
    if (m_useOdometry) {
        m_slam.addRequestNavDataCallback(&outside_lpslam_RequestNavDataCallback, this);
    }

    m_slam.start();

    setSlamStarted(true);
    RCLCPP_INFO(get_logger(), "LpSlam processing starting");
}

// according to https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/OccupancyGrid.msg
// http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html
void LpSlamNode::publishOccMap()
{
    if (!isSlamStarted()) {
        return;
    }

    if (m_occGridPublisher->get_subscription_count() == 0) {
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Start publishing occupancy map");
    const auto rawSizeNeeded = m_slam.mappingGetMapRawSize();

    nav_msgs::msg::OccupancyGrid occGridMessage;
    occGridMessage.data.resize(rawSizeNeeded);

    const auto mapInfo = m_slam.mappingGetMapRaw(occGridMessage.data.data(), occGridMessage.data.size());
    const auto exportCount = mapInfo.y_cell_count * mapInfo.x_cell_count;
    RCLCPP_DEBUG(get_logger(), "exported %u occupancy map points", exportCount);

    occGridMessage.info.height = mapInfo.y_cell_count;
    occGridMessage.info.width = mapInfo.x_cell_count;
    occGridMessage.info.resolution = mapInfo.x_cell_size;
    occGridMessage.info.origin.position.x = mapInfo.x_origin;
    occGridMessage.info.origin.position.y = mapInfo.y_origin;

    RCLCPP_DEBUG(get_logger(), "occupancy map origin (%f, %f)", occGridMessage.info.origin.position.x,
        occGridMessage.info.origin.position.y);

    //  account for higher placement of camera
    occGridMessage.info.origin.position.z = -1.15;

    occGridMessage.header.frame_id = m_map_frame_id;
    occGridMessage.header.stamp = this->now();

    m_occGridPublisher->publish(std::move(occGridMessage));
    m_mapMetaDataPublisher->publish(std::move(occGridMessage.info));
    RCLCPP_DEBUG(get_logger(), "Finished publishing occupancy map");
}

void LpSlamNode::publishPointCloud()
{
    if (!isSlamStarted()) {
        return;
    }

    if (m_pointcloudPublisher->get_subscription_count() == 0) {
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Starting to publish feature points");

    // can do that because the LpSlam structure has exactly three floats
    // might change in the future
    if (sizeof(LpSlamFeatureEntry) != (sizeof(float) * 3))
    {
        RCLCPP_ERROR(get_logger(), "Assumption about LpSlamFeatureEntry struct size not correct");
        return;
    }

    // ignored by the API anyways
    LpSlamMapBoundary bounds;
    std::vector<LpSlamFeatureEntry> featurePointBuffer;
    auto featureSize = m_slam.mappingGetFeaturesCount(bounds);

    // limit the max points for now
    featureSize = std::min(featureSize, std::size_t(50000));
    featurePointBuffer.resize(featureSize);

    // transform from LpSlam coords to ROS
    //transformation lpslam -> ROS is
    // z_ros = x_lpslam
    // x_ros = z_lpslam
    // y_ros = -y_lpslam

    LpSlamMatrix9x9 lpslam_to_ros = {
        0.0,  0.0,  1.0,
        0.0, -1.0,  0.0,
        1.0,  0.0,  0.0
    };

    auto outputSize = m_slam.mappingGetFeatures(bounds, featurePointBuffer.data(),
                                               featureSize, lpslam_to_ros);

    RCLCPP_DEBUG_STREAM(get_logger(), "Streaming out " << outputSize << " feature points");
    featurePointBuffer.resize(outputSize);

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
    std::memcpy(cloud_msg_ptr, (float*)featurePointBuffer.data(), outputSize * sizeof(LpSlamFeatureEntry));

    m_pointcloudPublisher->publish(std::move(cloud_msg));
    RCLCPP_DEBUG(get_logger(), "Finished publishing feature points");
}

bool LpSlamNode::check_and_dispatch(
    const sensor_msgs::msg::Image::SharedPtr this_msg,
    std::vector<uint8_t> &otherImageBuffer,
    std::optional<builtin_interfaces::msg::Time> &otherTimestamp,
    std::mutex &otherMutex,
    bool isLeft)
{
    std::scoped_lock lock(otherMutex);

    // is the cached image close in time what we already got ?
    if (!otherTimestamp.has_value())
    {
        return false;
    }

    uint64_t diff_seconds = otherTimestamp->sec - this_msg->header.stamp.sec;
    uint64_t diff_nanoseconds = otherTimestamp->nanosec - this_msg->header.stamp.nanosec;

    double dt = diff_seconds + std::pow(10, -9) * double(diff_nanoseconds);

    if (std::abs(dt) > m_timespanMatch)
    {
        return false;
    }

    // dispatch both images
    LpSlamImageDescription imgDesc;
    imgDesc.structure = LpSlamImageStructure_Stereo_TwoBuffer;
    if (this_msg->encoding == "mono8" ) {
        imgDesc.format = LpSlamImageFormat_8UC1;
    } else {
        imgDesc.format = LpSlamImageFormat_8UC3;
    }
    imgDesc.image_conversion = LpSlamImageConversion_None;
    imgDesc.height = this_msg->height;
    imgDesc.width = this_msg->width;
    imgDesc.imageSize = this_msg->step * this_msg->height;
    imgDesc.hasRosTimestamp = 1;
    imgDesc.rosTimestamp.seconds = this_msg->header.stamp.sec;
    imgDesc.rosTimestamp.nanoseconds = this_msg->header.stamp.nanosec;

    // slam timestamp
    uint64_t slam_ts = std::chrono::high_resolution_clock::now().time_since_epoch().count();

    // forward to library
    m_slam.addStereoImageFromBuffer(0,
                                    // timestamp
                                    slam_ts,
                                    isLeft ? this_msg->data.data() : otherImageBuffer.data(),
                                    isLeft ? otherImageBuffer.data() : this_msg->data.data(),
                                    imgDesc);
    otherTimestamp.reset();
    return true;
}

// Callbacks
void LpSlamNode::image_callback_right(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (check_and_dispatch(msg, m_leftImageBuffer, m_leftImageTimestamp, m_leftImageMutex, false))
    {
        return;
    }

    {
        std::scoped_lock lock(m_rightImageMutex);
        m_rightImageBuffer.resize(msg->step * msg->height);
        std::memcpy(m_rightImageBuffer.data(), msg->data.data(),
                    m_rightImageBuffer.size());
        m_rightImageTimestamp = msg->header.stamp;
    }
}

void LpSlamNode::image_callback_left(const sensor_msgs::msg::Image::SharedPtr msg)
{
    const std::string camera_encoding = getCameraEncoding();
    if (camera_encoding.empty()) {
        setCameraEncoding(msg->encoding);
    }

    if (check_and_dispatch(msg, m_rightImageBuffer, m_rightImageTimestamp, m_rightImageMutex, true))
    {
        return;
    }
    {
        std::scoped_lock lock(m_leftImageMutex);
        m_leftImageBuffer.resize(msg->step * msg->height);
        std::memcpy(m_leftImageBuffer.data(), msg->data.data(),
                    m_leftImageBuffer.size());
        m_leftImageTimestamp = msg->header.stamp;
    }
}

void LpSlamNode::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // This can be used to transform laser scan into camera frame
    // http://docs.ros.org/en/latest/api/tf2_ros/html/c++/classtf2__ros_1_1BufferInterface.html#a12d0bda7d7533f199e94dae4bb46ba7a
    // Use this method to lookup the transform of the laser data at time t_l to the camera pose at t_c

    RCLCPP_DEBUG(get_logger(), "Received %lu laser scans with min %f max %f angle start %f angle end %f",
        msg->ranges.size(), msg->range_min, msg->range_max, msg->angle_min, msg->angle_max);

    // wait up to 100 ms for transform to arrive, parameter is nanoseconds
    auto transformTimeout = rclcpp::Duration::from_nanoseconds(0.1 * std::pow(10.0, 9.0));

    std::optional<geometry_msgs::msg::TransformStamped> odomTransform;
    try {
        // first: target frame
        // second: source frame
        // get the transformation for the laser scan's time, get odometry frame
        // because this information is not really used by LPSLAM and its always available,
        // in contrast to map frame transform
        odomTransform = m_tfBuffer->lookupTransform(m_odom_frame_id, m_laserscan_frame_id,
            msg->header.stamp, transformTimeout);
    } catch (tf2::LookupException &ex) {
        RCLCPP_ERROR(get_logger(), "Cannot process laser scan because map -> laser transformation not available: %s", ex.what());
        return;
    } catch (tf2::ConnectivityException &ex) {
        RCLCPP_ERROR(get_logger(), "Cannot process laser scan because map -> laser transformation not available: %s", ex.what());
        return;
    } catch (tf2::ExtrapolationException &ex) {
        RCLCPP_ERROR(get_logger(), "Cannot process laser scan because map -> laser transformation not available: %s", ex.what());
        return;
    } catch (tf2::InvalidArgumentException &ex) {
        RCLCPP_ERROR(get_logger(), "Cannot process laser scan because map -> laser transformation not available: %s", ex.what());
        return;
    }

    auto lpstate = transformToLpSlamGlobalState(odomTransform.value());
    lpstate.has_ros_timestamp = true;
    lpstate.ros_timestamp = rosTimeToLpSlam(msg->header.stamp);

    // use the odomtry transform to get the global coordinates because its more reliable and faster
    // than the pure optical measurement
    // LaserScan data is obviously in laser frame
    m_slam.mappingAddLaserScan(lpstate, msg->ranges.data(), msg->ranges.size(),
        msg->range_min, msg->range_max,
        msg->angle_min, msg->angle_max,
        msg->angle_increment, 3.0);

    RCLCPP_DEBUG(get_logger(), "Laser data dispatched");
}

void LpSlamNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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
    // Update LP-SLAM config file with prepared OpenVSLAM one
    if (!make_lpslam_config())
    {
        return;
    }

    // All configurations were prepared. Starting SLAM.
    startSlam();

    // Once camera was configured, no more actions needed here
    m_cameraConfigured = true;
}

bool LpSlamNode::make_lpslam_config()
{
    if (m_openVSlamYaml.empty())
    {
        RCLCPP_ERROR(get_logger(), "No OpenVSLAM config-file prepared");
        return false;
    }

    // Write formed YAML to file
    m_lpSlamJson = std::tmpnam(nullptr);
    m_lpSlamJson += ".json";
    try {
        // Open m_configFile for reading
        std::ifstream in_json(m_configFile);
        std::string ln;
        // And m_lpSlamJson for writing
        std::ofstream out_json(m_lpSlamJson);
        size_t config_pos;
        // Replace "configFromFile" field with m_openVSlamYaml
        while (std::getline(in_json, ln))
        {
            config_pos = ln.find("configFromFile");
            if (config_pos != std::string::npos)
            {
                ln = ln.substr(0, config_pos) +
                    "configFromFile\": \"" + m_openVSlamYaml + "\",";
            }
            out_json << ln << std::endl;
        }
        in_json.close();
        out_json.close();
    } catch (std::exception & e) {
        RCLCPP_ERROR(
            get_logger(), "Error while processing %s -> to %s: %s",
            m_configFile.c_str(), m_lpSlamJson.c_str(), e.what());
        return false;
    }
    RCLCPP_INFO(get_logger(), "%s LP-SLAM config has been prepared", m_lpSlamJson.c_str());
    return true;
}

void LpSlamNode::stopSlam()
{
    m_slam.stop();
}
 
} // namespace lpslam_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lpslam_components::LpSlamNode)