//===========================================================================//
//
// Copyright (C) 2021 LP-Research Inc.
//
//===========================================================================//

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <lpslam_interfaces/msg/lp_slam_status.hpp>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "LpSlamManager.h"

#include <cstring>
#include <memory>
#include <string>
#include <iostream>
#include <optional>
#include <mutex>
#include <algorithm>
#include <chrono>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>

/*
nice explainer:
https://www.stereolabs.com/docs/ros2/video/
*/

void outside_lpslam_OnReconstructionCallback(LpSlamGlobalStateInTime const &state_in_time, void *lpslam_object);

LpSlamRequestNavDataResult outside_lpslam_RequestNavDataCallback(LpSlamROSTimestamp for_ros_time,
    LpSlamGlobalStateInTime * odometry,
    LpSlamGlobalStateInTime * map,
    void *);

LpSlamRequestNavTransformation outside_lpslam_RequestNavTransformationCallback(LpSlamROSTimestamp ros_time,
    LpSlamNavDataFrame from_frame,
    LpSlamNavDataFrame to_frame,
    void * lpslam_node);

class LpSlamNode : public rclcpp::Node
{
public:
    LpSlamNode() : Node("lpslam_node")
    {
        RCLCPP_INFO(get_logger(), "LpSlam node started");

        m_configFile = this->declare_parameter<std::string>("lpslam_config", "");
        m_lpSlamLogFile = this->declare_parameter<std::string>("lpslam_log_file", "lpslam.log");
        m_writeLpSlamLog = this->declare_parameter<bool>("write_lpslam_log", false);
        m_isStereoCamera = this->declare_parameter<bool>("stereo_camera", true);
        m_consumeLaser = this->declare_parameter<bool>("consume_laser", true);
        m_isCombinedStereoImage = this->declare_parameter<bool>("combined_stereo_image", false);
        m_timespanMatch = this->declare_parameter<double>("timespan_match", 0.010); // 10 ms mach distance
        m_map_frame_id = this->declare_parameter<std::string>("map_frame_id", "map");
        m_camera_frame_id = this->declare_parameter<std::string>("camera_frame_id", "camera_link");
        m_odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "odom");
        m_base_frame_id = this->declare_parameter<std::string>("base_frame_id", "base_link");
        m_laserscan_frame_id = this->declare_parameter<std::string>("laserscan_frame_id", "laser");

        auto useSimTime = this->get_parameter( "use_sim_time" ).as_bool();
        RCLCPP_INFO(get_logger(), "LpSlamNode is using sim time: %s", useSimTime ? "yes": "no");

        const std::string left_image_topic = this->declare_parameter<std::string>(
            "left_image_topic", "left_image_raw");
        const std::string right_image_topic = this->declare_parameter<std::string>(
            "right_image_topic", "right_image_raw");
        const auto map_name = this->declare_parameter<std::string>("map_name", "/map");
        const std::string laserscan_topic = this->declare_parameter<std::string>("laserscan_topic", "scan");

        const std::string pointcloud_topic = this->declare_parameter<std::string>("pointcloud_topic", "slam_features");
        const auto pointcloud_rate = this->declare_parameter<int>("pointcloud_rate", 10);
        const auto map_rate = this->declare_parameter<int>("map_rate", 5);

        const std::string lpslam_status_topic = this->declare_parameter<std::string>(
            "lpslam_status_topic", "lpslam_status");

        m_use_odometry = this->declare_parameter<bool>("use_odometry", true);

        const auto transformToleranceFloat = this->declare_parameter<float>("transform_tolerance", 0.5);
        m_transform_tolerance = tf2::durationFromSec(transformToleranceFloat);

        initTransforms();

        // allow for relaxed QoS for laser scan in order to match
        // the topic settings
        rclcpp::QoS laser_qos(5);
        laser_qos.keep_last(5);
        laser_qos.best_effort();
        laser_qos.durability_volatile();

        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        
        if (m_consumeLaser) { 
            m_laserScanSubsription = this->create_subscription<sensor_msgs::msg::LaserScan>(
                laserscan_topic, default_qos,
                std::bind(&LpSlamNode::laserscan_callback, this, std::placeholders::_1));
        }

        // allow for relaxed QoS for image in order to match
        // the topic settings
        rclcpp::QoS video_qos(10);
        video_qos.keep_last(10);
        video_qos.best_effort();
        video_qos.durability_volatile();

        m_leftImageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            left_image_topic, video_qos,
            std::bind(&LpSlamNode::image_callback_left, this, std::placeholders::_1));

        if (m_isStereoCamera && !m_isCombinedStereoImage)
        {
            m_rightImageSubscription = this->create_subscription<sensor_msgs::msg::Image>(
                right_image_topic, video_qos,
                std::bind(&LpSlamNode::image_callback_right, this, std::placeholders::_1));
        }

        m_pointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 1);

        m_occGridPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_name,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        m_mapMetaDataPublisher = this->create_publisher<nav_msgs::msg::MapMetaData>(map_name + "_metadata",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        m_slamStatusPublisher = this->create_publisher<lpslam_interfaces::msg::LpSlamStatus>(
            lpslam_status_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        if (m_writeLpSlamLog)
        {
            m_slam.logToFile(m_lpSlamLogFile.c_str());
        }
        m_slam.setLogLevel(LpSlamLogLevel_Debug);
        RCLCPP_INFO_STREAM(get_logger(), "Loading LpSlam configuration from " << m_configFile);
        m_slam.readConfigurationFile(m_configFile.c_str());

        m_slam.addOnReconstructionCallback(&outside_lpslam_OnReconstructionCallback, this);
        m_slam.addRequestNavTransformation(&outside_lpslam_RequestNavTransformationCallback, this);
        if (m_use_odometry) {
            m_slam.addRequestNavDataCallback(&outside_lpslam_RequestNavDataCallback, this);
        }

        m_slam.start();
        RCLCPP_INFO(get_logger(), "LpSlam processing starting");

        // start publishing point cloud
        auto ros_clock = rclcpp::Clock::make_shared();
        m_pointcloud_timer = rclcpp::create_timer(this, ros_clock, rclcpp::Duration(pointcloud_rate, 0),
                                                  [this]() {
                                                      this->publishPointCloud();
                                                  });
        m_occmap_timer = rclcpp::create_timer(this, ros_clock, rclcpp::Duration(map_rate, 0),
                                                  [this]() {
                                                      this->publishOccMap();
                                                  });

        m_slamstatus_timer = rclcpp::create_timer(this, ros_clock, rclcpp::Duration(1, 0),
                                                  [this]() {
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

    virtual ~LpSlamNode()
    {
        RCLCPP_INFO(get_logger(), "shutting down LpSlam");
        m_pointcloud_timer.reset();
        m_occmap_timer.reset();
        m_slamstatus_timer.reset();
        m_slam.stop();
        RCLCPP_INFO(get_logger(), "shutting down LpSlam complete");
    }

private:

    // according to https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/OccupancyGrid.msg
    // http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html
    void publishOccMap()
    {
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

    void publishPointCloud()
    {
        if (m_pointcloudPublisher->get_subscription_count() == 0) {
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Starting to publish feature points");

        // ignored by the API anyways
        LpSlamMapBoundary bounds;
        auto featureSize = m_slam.mappingGetFeaturesCount(bounds);

        // limit the max points for now
        featureSize = std::min(featureSize, std::size_t(50000));
        m_featurePointBuffer.resize(featureSize);

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

        auto outputSize = m_slam.mappingGetFeatures(bounds, m_featurePointBuffer.data(),
                                                   featureSize, lpslam_to_ros);

        RCLCPP_DEBUG_STREAM(get_logger(), "Streaming out " << outputSize << " feature points");
        m_featurePointBuffer.resize(outputSize);

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

        // can do that because the LpSlam structure has exactly three floats
        // might change in the future
        if (sizeof(LpSlamFeatureEntry) != (sizeof(float) * 3))
        {
            RCLCPP_ERROR(get_logger(), "Assumption about LpSlamFeatureEntry struct size not correct");
            return;
        }

        float* cloud_msg_ptr = (float*)(&cloud_msg->data[0]);
        std::memcpy(cloud_msg_ptr, (float*)m_featurePointBuffer.data(), outputSize * sizeof(LpSlamFeatureEntry));

        m_pointcloudPublisher->publish(std::move(cloud_msg));
        RCLCPP_DEBUG(get_logger(), "Finished publishing feature points");
    }

private:
    void initTransforms()
    {
        // Initialize transform listener and broadcaster
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        m_tfBuffer->setCreateTimerInterface(timer_interface);
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    bool check_and_dispatch(const sensor_msgs::msg::Image::SharedPtr this_msg,
                            std::vector<uint8_t> &otherImageBuffer, std::optional<builtin_interfaces::msg::Time> &otherTimestamp,
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
    void image_callback_right(const sensor_msgs::msg::Image::SharedPtr msg)
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

    void image_callback_left(const sensor_msgs::msg::Image::SharedPtr msg)
    {
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

    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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

public:
    // needs to be public because its called from the outside
    void lpslam_OnReconstructionCallback(LpSlamGlobalStateInTime const &state_in_time)
    {
        //transformation lpslam -> ROS is
        // z_ros = x_lpslam
        // x_ros = z_lpslam
        // y_ros = -y_lpslam

        auto const &state = state_in_time.state;

        if (!state.valid)  {
            RCLCPP_DEBUG(get_logger(), "SLAM Tracking state invalid, wont use it.");

               // send out the last transform we know
            if (m_lastTransform.has_value())  {
                auto msg = m_lastTransform.value();
                msg.header.stamp = this->now() + m_transform_tolerance;
                m_tfBroadcaster->sendTransform(msg);
                RCLCPP_INFO(get_logger(), "Send out old transform because new one does not exist.");
            }
            return;
        }

        // subtracting base to odom from map to base and send map to odom instead
        geometry_msgs::msg::PoseStamped odom_to_map;
        tf2::Transform new_tf;
        tf2::TimePoint transform_expiration;

        // relevant lines:
        // https://github.com/SteveMacenski/slam_toolbox/blob/foxy-devel/src/slam_toolbox_common.cpp#L386
        computeAndPublishTransform(state_in_time);
    }

    LpSlamRequestNavTransformation lpslam_RequestNavTransformationCallback(LpSlamROSTimestamp ros_time,
        LpSlamNavDataFrame from_frame,
        LpSlamNavDataFrame to_frame) {

        LpSlamRequestNavTransformation invalid_res;
        invalid_res.valid = false;

        RCLCPP_DEBUG(get_logger(), "Requested nav transform for seconds %d nseconds %ld", ros_time.seconds, ros_time.nanoseconds);
        
        auto lmdMapName = [this](LpSlamNavDataFrame frame_enum )  {
            if (frame_enum == LpSlamNavDataFrame_Camera) {
                return m_camera_frame_id;
            } else if (frame_enum == LpSlamNavDataFrame_Laser) {
                return m_laserscan_frame_id;
            } else if (frame_enum == LpSlamNavDataFrame_Odometry) {
                return m_odom_frame_id;
            } else {
                return std::string("");
            }
        };

        const auto transformTimeout = rclcpp::Duration::from_nanoseconds(0.1 * std::pow(10.0, 9.0));

        std::optional<geometry_msgs::msg::TransformStamped> transform;
        auto from_frame_id = lmdMapName(from_frame);
        auto to_frame_id = lmdMapName(to_frame);

        if (to_frame_id.empty() || from_frame_id.empty()) {
            RCLCPP_ERROR(get_logger(), "LpSlam fram cannot be converted to ROS frame");
            return invalid_res;
        }

        const auto cpp_ros_time = lpSlamRosTime(ros_time);

        try {
            transform = m_tfBuffer->lookupTransform(to_frame_id, from_frame_id, cpp_ros_time,
                transformTimeout);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Cannot get transformation for LpSlam Callback %s", ex.what());
            return invalid_res;
        }

        const auto lpstate = transformToLpSlamGlobalState(transform.value());
        return lpstate.state;
    }

    LpSlamRequestNavDataResult lpslam_RequestNavDataCallback(LpSlamROSTimestamp for_ros_time,
        LpSlamGlobalStateInTime * odometry,
        LpSlamGlobalStateInTime * map ) {

        RCLCPP_DEBUG(get_logger(), "Requested nav data for time %i %lu", for_ros_time.seconds, for_ros_time.nanoseconds);

        auto navTime = lpSlamRosTime(for_ros_time);

        // wait up to 100 ms for transform to arrive, parameter is nanoseconds
        auto transformTimeout = rclcpp::Duration::from_nanoseconds(0.1 * std::pow(10.0, 9.0));

        std::optional<geometry_msgs::msg::TransformStamped> odomTransform;
        try {
            // first: target frame
            // second: source frame
            // lookup the full transformation all the way to the camera
            // this will include the odometry transformation whichis the relevant part here !
            // because the map -> odom transformation is quite stable over time this
            // allows us to provide global position input, even if the tracking is lost
            odomTransform = m_tfBuffer->lookupTransform(m_odom_frame_id, m_camera_frame_id, navTime,
                transformTimeout);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Cannot get odometry transformation: %s", ex.what());
        }
        
        std::optional<geometry_msgs::msg::TransformStamped> mapTransform;
        try {
            // first: target frame
            // second: source frame
            // lookup the full transformation all the way to the camera
            // this will include the odometry transformation whichis the relevant part here !
            // because the map -> odom transformation is quite stable over time this
            // allows us to provide global position input, even if the tracking is lost
            mapTransform = m_tfBuffer->lookupTransform(m_map_frame_id, m_camera_frame_id, navTime,
                transformTimeout);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Cannot get map transformation: %s", ex.what());
        }

        // todo handle  case when there is no map transform
        if (odomTransform.has_value()) {
            RCLCPP_DEBUG(get_logger(), "odometry transformation to user: %f %f %f  Rot %f %f %f %f",
                odomTransform->transform.translation.x,
                odomTransform->transform.translation.y,
                odomTransform->transform.translation.z,
                odomTransform->transform.rotation.w,
                odomTransform->transform.rotation.x,
                odomTransform->transform.rotation.y,
                odomTransform->transform.rotation.z);

            *odometry = transformToLpSlamGlobalState(odomTransform.value());

        }

        // todo handle  case when there is no map transform
        if (mapTransform.has_value()) {
            RCLCPP_DEBUG(get_logger(), "map transformation to user: %f %f %f  Rot %f %f %f %f",
                mapTransform->transform.translation.x,
                mapTransform->transform.translation.y,
                mapTransform->transform.translation.z,
                mapTransform->transform.rotation.w,
                mapTransform->transform.rotation.x,
                mapTransform->transform.rotation.y,
                mapTransform->transform.rotation.z);

            *map = transformToLpSlamGlobalState(mapTransform.value());

        }

        if (mapTransform.has_value() && odomTransform.has_value()) {
            return LpSlamRequestNavDataResult_OdomAndMap;
        }

        if (mapTransform.has_value() && !odomTransform.has_value()) {
            return LpSlamRequestNavDataResult_MapOnly;
        }

        if (!mapTransform.has_value() && odomTransform.has_value()) {
            return LpSlamRequestNavDataResult_OdomOnly;
        }

        // invalid odom
        return LpSlamRequestNavDataResult_None;
    }

    void computeAndPublishTransform(LpSlamGlobalStateInTime const & state_in_time) {
        auto const & state = state_in_time.state;

        if (state_in_time.has_ros_timestamp == 0) {
            RCLCPP_ERROR(get_logger(), "LPSLAM result does not contain ROS timestamp, can't use tracking result");
            return;
        }

        auto const imageTimestamp = lpSlamRosTime(state_in_time.ros_timestamp);

        tf2::Stamped<tf2::Transform> odom_to_map;
        tf2::Quaternion q(state.orientation.z, -state.orientation.y, state.orientation.x,
                            state.orientation.w);
        tf2::Vector3 p(state.position.z,-state.position.y, state.position.x);
        tf2::Vector3 p_rot = tf2::quatRotate(q, p);
        tf2::Stamped<tf2::Transform> cam_to_map(
            tf2::Transform(q, -p_rot), tf2_ros::fromMsg(imageTimestamp), m_camera_frame_id);

        try {
            geometry_msgs::msg::TransformStamped cam_to_map_msg, base_to_map_msg, odom_to_map_msg;

            // https://github.com/ros2/geometry2/issues/176
            // not working for some reason...
            // base_to_map_msg = tf2::toMsg(base_to_map);
            cam_to_map_msg.header.stamp = imageTimestamp;
            cam_to_map_msg.header.frame_id = cam_to_map.frame_id_;
            cam_to_map_msg.transform.translation.x = cam_to_map.getOrigin().getX();
            cam_to_map_msg.transform.translation.y = cam_to_map.getOrigin().getY();
            cam_to_map_msg.transform.translation.z = cam_to_map.getOrigin().getZ();
            cam_to_map_msg.transform.rotation = tf2::toMsg(cam_to_map.getRotation());

            m_tfBuffer->transform(cam_to_map_msg, base_to_map_msg, m_base_frame_id);

            odom_to_map_msg = m_tfBuffer->transform(base_to_map_msg, m_odom_frame_id);
            tf2::fromMsg(odom_to_map_msg, odom_to_map);
        } catch (tf2::TransformException & e) {
            RCLCPP_ERROR(get_logger(), "Transform from base_link to odom failed: %s",
            e.what());
            return;
        }

        // set map to odom for our transformation thread to publish
        tf2::Transform map_to_odom = tf2::Transform(tf2::Quaternion(odom_to_map.getRotation() ),
            tf2::Vector3(odom_to_map.getOrigin() ) ).inverse();

        geometry_msgs::msg::TransformStamped msg;
        msg.transform = tf2::toMsg(map_to_odom);
        msg.child_frame_id = m_odom_frame_id;
        msg.header.frame_id = m_map_frame_id;
        msg.header.stamp = imageTimestamp + m_transform_tolerance;


        RCLCPP_DEBUG(get_logger(), "Sending out new transform %f %f %f rot %f %f %f %f",
                   msg.transform.translation.x,
                    msg.transform.translation.y,
                    msg.transform.translation.z,
                    msg.transform.rotation.w,
                    msg.transform.rotation.x,
                    msg.transform.rotation.y,
                    msg.transform.rotation.z);

        m_lastTransform = msg;

        m_tfBroadcaster->sendTransform(msg);
    }

    LpSlamGlobalStateInTime transformToLpSlamGlobalState(geometry_msgs::msg::TransformStamped const& tf) {
        LpSlamGlobalStateInTime ros_state;
        // ignored by LpSlam at the moment
        ros_state.timestamp = 0;
        ros_state.state.position = {tf.transform.translation.z,
            -tf.transform.translation.y,
            tf.transform.translation.x,
            0.1, 0.1, 0.1};

        const auto qRot = tf2::Quaternion(tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);

        // LpSlam Convention needs an inverted quaternion
        const auto invRot = qRot.inverse();

        ros_state.state.orientation = {invRot.w(),
            invRot.z(),
            -invRot.y(),
            invRot.x(), 0.1};

        ros_state.state.valid = true;

        return ros_state;
    }

    rclcpp::Time lpSlamRosTime( LpSlamROSTimestamp const& ts ) const {
        // note: need to use the constructor with just nanoseconds because
        // it can use int64, the one taking seconds and nanosconds only takes
        // int32 nanoseconds, therefore cutting off large entries of
        // nano seconds
        return rclcpp::Time(ts.nanoseconds + ts.seconds * std::pow(10,9));
    }

    LpSlamROSTimestamp rosTimeToLpSlam( rclcpp::Time const& ts ) const {
        return LpSlamROSTimestamp{0, ts.nanoseconds()};
    }

    // publishers
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointcloudPublisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> m_occGridPublisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::MapMetaData>> m_mapMetaDataPublisher;
    std::shared_ptr<rclcpp::Publisher<lpslam_interfaces::msg::LpSlamStatus>> m_slamStatusPublisher;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laserScanSubsription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_leftImageSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_rightImageSubscription;

    // TF2 handlers
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
    std::vector<LpSlamFeatureEntry> m_featurePointBuffer;

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

    // frame ids
    std::string m_map_frame_id;
    std::string m_camera_frame_id;
    std::string m_odom_frame_id;
    std::string m_laserscan_frame_id;
    std::string m_base_frame_id;

    std::optional<geometry_msgs::msg::TransformStamped> m_lastTransform;

    // configuration variables
    std::string m_configFile;
    std::string m_lpSlamLogFile;
    bool m_use_odometry;
    bool m_writeLpSlamLog;
    bool m_isStereoCamera;
    bool m_consumeLaser;
    double m_timespanMatch;
    // means that both stereo images are in one physical image
    // transferred via the ROS topic left_image_raw
    bool m_isCombinedStereoImage;
    tf2::Duration m_transform_tolerance;

    // the godly SlamManager
    LpSlamManager m_slam;
};

void outside_lpslam_OnReconstructionCallback(LpSlamGlobalStateInTime const &state_in_time, void *lpslam_node)
{
    static_cast<LpSlamNode *>(lpslam_node)->lpslam_OnReconstructionCallback(state_in_time);
}

LpSlamRequestNavDataResult outside_lpslam_RequestNavDataCallback(LpSlamROSTimestamp for_ros_time,
    LpSlamGlobalStateInTime * odometry,
    LpSlamGlobalStateInTime * map,
    void * lpslam_node){
    return static_cast<LpSlamNode *>(lpslam_node)->lpslam_RequestNavDataCallback(for_ros_time,
        odometry, map);
}

LpSlamRequestNavTransformation outside_lpslam_RequestNavTransformationCallback(LpSlamROSTimestamp ros_time,
    LpSlamNavDataFrame from_frame,
    LpSlamNavDataFrame to_frame,
    void * lpslam_node) {
    return static_cast<LpSlamNode *>(lpslam_node)->lpslam_RequestNavTransformationCallback(ros_time,
        from_frame, to_frame);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LpSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
