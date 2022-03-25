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

#include "LpBaseNode.hpp"

/*
nice explainer:
https://www.stereolabs.com/docs/ros2/video/
*/

namespace lpslam_components
{
    LpBaseNode::LpBaseNode(const rclcpp::NodeOptions & options) : Node("lpslam_node", options)
{
    if (!setParameters()) {
        return;
    }

    initTransforms();
}

bool LpBaseNode::setParameters()
{
    auto useSimTime = this->get_parameter( "use_sim_time" ).as_bool();
    RCLCPP_INFO(get_logger(), "VSLAM node is using sim time: %s", useSimTime ? "yes": "no");

    // general VSLAM parameters
    m_vSlamMethod = "OpenVSLAM";
    m_slamStarted = false;

    // source switches
    m_useOdometry = this->declare_parameter<bool>("use_odometry", true);
    m_consumeLaser = this->declare_parameter<bool>("consume_laser", true);
    m_isStereoCamera = this->declare_parameter<bool>("stereo_camera", true);
    m_isCombinedStereoImage = this->declare_parameter<bool>("combined_stereo_image", false);

    // frame ids
    m_map_frame_id = this->declare_parameter<std::string>("map_frame_id", "map");
    m_camera_frame_id = this->declare_parameter<std::string>("camera_frame_id", "camera_link");
    m_odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "odom");
    m_base_frame_id = this->declare_parameter<std::string>("base_frame_id", "base_link");
    m_laserscan_frame_id = this->declare_parameter<std::string>("laserscan_frame_id", "laser");

    // topics and rates
    m_qosReliability = this->declare_parameter<std::string>(
        "qos_reliability", "best_effort");
    m_laserscanTopic = this->declare_parameter<std::string>("laserscan_topic", "scan");
    m_leftImageTopic = this->declare_parameter<std::string>(
        "left_image_topic", "left_image_raw");
    m_rightImageTopic = this->declare_parameter<std::string>(
        "right_image_topic", "right_image_raw");
    m_cameraInfoTopic = this->declare_parameter<std::string>(
        "camera_info_topic", "");
    m_cameraFps = this->declare_parameter<double>("camera_fps", 5.0);
    m_pointcloudTopic = this->declare_parameter<std::string>("pointcloud_topic", "slam_features");
    m_pointcloudRate = this->declare_parameter<int>("pointcloud_rate", 10);
    m_mapName = this->declare_parameter<std::string>("map_name", "/map");
    m_mapRate = this->declare_parameter<int>("map_rate", 5);

    const auto transformToleranceFloat = this->declare_parameter<float>("transform_tolerance", 0.5);
    m_transform_tolerance = tf2::durationFromSec(transformToleranceFloat);

    // OpenVSLAM parameters
    m_openVSlam_maxNumKeypoints = this->declare_parameter<int>(
        m_vSlamMethod + "." + "max_num_keypoints", 1000);
    m_openVSlam_iniMaxNumKeypoints = this->declare_parameter<int>(
        m_vSlamMethod + "." + "ini_max_num_keypoints", 2000);
    m_openVSlam_scaleFactor = this->declare_parameter<double>(
        m_vSlamMethod + "." + "scale_factor", 1.2);
    m_openVSlam_numLevels = this->declare_parameter<int>(
        m_vSlamMethod + "." + "num_levels", 8);
    m_openVSlam_iniFastThreshold = this->declare_parameter<int>(
        m_vSlamMethod + "." + "ini_fast_threshold", 20);
    m_openVSlam_minFastThreshold = this->declare_parameter<int>(
        m_vSlamMethod + "." + "min_fast_threshold", 7);
    m_openVSlam_depthThreshold = this->declare_parameter<double>(
        m_vSlamMethod + "." + "depth_threshold", 40.0);
    m_openVSlam_depthmapFactor = this->declare_parameter<double>(
        m_vSlamMethod + "." + "depthmap_factor", 1.0);
    m_openVSlam_mappingBaselineDistThr = this->declare_parameter<double>(
        m_vSlamMethod + "." + "mapping_baseline_dist_thr", 0.1);
    m_openVSlam_pangolinViewerFps  = this->declare_parameter<double>(
        m_vSlamMethod + "." + "pangolin_viewer_fps", 10.0);

    m_openVSlamYaml = this->declare_parameter<std::string>(
        m_vSlamMethod + "." + "config_yaml", "");
    m_cameraConfigured = false;

    m_openVSlam_cameraEncoding = "";

    return true;
}

void LpBaseNode::initTransforms()
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

// needs to be public because its called from the outside
void LpBaseNode::lpslam_OnReconstructionCallback(LpSlamGlobalStateInTime const &state_in_time)
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

    // relevant lines:
    // https://github.com/SteveMacenski/slam_toolbox/blob/foxy-devel/src/slam_toolbox_common.cpp#L386
    computeAndPublishTransform(state_in_time);
}

LpSlamRequestNavTransformation LpBaseNode::lpslam_RequestNavTransformationCallback(LpSlamROSTimestamp ros_time,
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

    const auto cpp_ros_time = lpSlamToRosTime(ros_time);

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

LpSlamRequestNavDataResult LpBaseNode::lpslam_RequestNavDataCallback(LpSlamROSTimestamp for_ros_time,
    LpSlamGlobalStateInTime * odometry,
    LpSlamGlobalStateInTime * map ) {

    RCLCPP_DEBUG(get_logger(), "Requested nav data for time %i %lu", for_ros_time.seconds, for_ros_time.nanoseconds);

    auto navTime = lpSlamToRosTime(for_ros_time);

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

void LpBaseNode::computeAndPublishTransform(LpSlamGlobalStateInTime const & state_in_time) {
    auto const & state = state_in_time.state;

    if (state_in_time.has_ros_timestamp == 0) {
        RCLCPP_ERROR(get_logger(), "LPSLAM result does not contain ROS timestamp, can't use tracking result");
        return;
    }

    auto const imageTimestamp = lpSlamToRosTime(state_in_time.ros_timestamp);

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

bool LpBaseNode::make_openvslam_config(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // Form OpenVSLAM config YAML
    YAML::Node configNode;

    if (msg->distortion_model != "plumb_bob") {
        RCLCPP_ERROR(
            get_logger(),
            "%s distortion model is not supported",
            msg->distortion_model.c_str());
        return false;
    }

    configNode["Camera"]["name"] = "LpSlam";
    if (m_isStereoCamera)
    {
        configNode["Camera"]["setup"] = "stereo";
    } else {
        configNode["Camera"]["setup"] = "monocular";
    }
    configNode["Camera"]["model"] = "perspective";

    configNode["Camera"]["fx"] = msg->k[0];  // k[0,0]
    configNode["Camera"]["fy"] = msg->k[4];  // k[1,1]
    configNode["Camera"]["cx"] = msg->k[2];  // k[0,2]
    configNode["Camera"]["cy"] = msg->k[5];  // k[1,2]

    // For perspective model
    configNode["Camera"]["k1"] = 0.0;
    configNode["Camera"]["k2"] = 0.0;
    configNode["Camera"]["k3"] = 0.0;
    configNode["Camera"]["p1"] = 0.0;
    configNode["Camera"]["p2"] = 0.0;

    if (m_isStereoCamera) {
        configNode["Camera"]["focal_x_baseline"] = -msg->p[3];  // -p[0,3]
    }

    configNode["Camera"]["fps"] = m_cameraFps;
    configNode["Camera"]["cols"] = msg->width;
    configNode["Camera"]["rows"] = msg->height;

    // Trying to get Camera.color_order
    if (!get_camera_color_order(configNode)) {
        return false;
    }

    configNode["Feature"]["max_num_keypoints"] = m_openVSlam_maxNumKeypoints;
    configNode["Feature"]["ini_max_num_keypoints"] = m_openVSlam_iniMaxNumKeypoints;
    configNode["Feature"]["scale_factor"] = m_openVSlam_scaleFactor;
    configNode["Feature"]["num_levels"] = m_openVSlam_numLevels;
    configNode["Feature"]["ini_fast_threshold"] = m_openVSlam_iniFastThreshold;
    configNode["Feature"]["min_fast_threshold"] = m_openVSlam_minFastThreshold;

    configNode["depth_threshold"] = m_openVSlam_depthThreshold;
    configNode["depthmap_factor"] = m_openVSlam_depthmapFactor;

    configNode["Mapping"]["baseline_dist_thr"] = m_openVSlam_mappingBaselineDistThr;

    configNode["PangolinViewer"]["fps"] = m_openVSlam_pangolinViewerFps;

    // Write formed YAML to file
    m_openVSlamYaml = std::tmpnam(nullptr);
    m_openVSlamYaml += ".yaml";
    try {
        std::ofstream yaml(m_openVSlamYaml);
        yaml << configNode << std::endl;
        yaml.close();
    } catch (std::exception & e) {
        RCLCPP_ERROR(
            get_logger(), "Can not write OpenVSLAM config to %s: %s",
            m_openVSlamYaml.c_str(), e.what());
        return false;
    }
    RCLCPP_INFO(get_logger(), "%s OpenVSLAM config has been prepared", m_openVSlamYaml.c_str());
    return true;
}

bool LpBaseNode::get_camera_color_order(YAML::Node & configNode)
{
    const int max_try = 10;
    const int wait_ms = 200;

    for (int c = 0; c < max_try; c++) {
        const std::string camera_encoding = getCameraEncoding();
        if (!camera_encoding.empty()) {
            if (camera_encoding == sensor_msgs::image_encodings::RGB8) {
                configNode["Camera"]["color_order"] = "RGB";
                return true;
            } else if (camera_encoding == sensor_msgs::image_encodings::RGBA8) {
                configNode["Camera"]["color_order"] = "RGBA";
                return true;
            } else if (camera_encoding == sensor_msgs::image_encodings::BGR8) {
                configNode["Camera"]["color_order"] = "BGR";
                return true;
            } else if (camera_encoding == sensor_msgs::image_encodings::BGRA8) {
                configNode["Camera"]["color_order"] = "BGRA";
                return true;
            } else if (camera_encoding == sensor_msgs::image_encodings::MONO8) {
                configNode["Camera"]["color_order"] = "Gray";
                return true;
            } else {
                RCLCPP_ERROR(
                    get_logger(),
                    "%s camera encoding is not supported by OpenVSLAM",
                    camera_encoding.c_str());
                return false;
            }
        }
        // Wait for some time
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }

    RCLCPP_WARN(get_logger(), "Can not obtain camera encoding within %i ms", wait_ms * max_try);
    return false;
}

LpSlamGlobalStateInTime LpBaseNode::transformToLpSlamGlobalState(geometry_msgs::msg::TransformStamped const& tf) const
{
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

rclcpp::Time LpBaseNode::lpSlamToRosTime( LpSlamROSTimestamp const& ts ) const
{
    // note: need to use the constructor with just nanoseconds because
    // it can use int64, the one taking seconds and nanosconds only takes
    // int32 nanoseconds, therefore cutting off large entries of
    // nano seconds
    return rclcpp::Time(ts.nanoseconds + ts.seconds * std::pow(10,9));
}

LpSlamROSTimestamp LpBaseNode::rosTimeToLpSlam( rclcpp::Time const& ts ) const
{
    return LpSlamROSTimestamp{0, ts.nanoseconds()};
}

    
} // namespace lpslam_components

