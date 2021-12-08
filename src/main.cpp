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

#ifndef USE_OPENVSLAM_DIRECTLY
#include "LpSlamNode.hpp"
#define VSLAM LpSlamNode
#else
#include "OpenVSLAMNode.hpp"
#define VSLAM OpenVSLAMNode
#endif  // USE_OPENVSLAM_DIRECTLY

// Signal handler for no-destructor issue:
// https://answers.ros.org/question/364045/publish-message-on-destructor-works-only-on-rclpy
// https://stackoverflow.com/questions/58644994/destructor-of-shared-ptr-object-never-called
// https://github.com/ros2/rclcpp/issues/317
std::shared_ptr<VSLAM> node_;
void signal_handler(int /*sig_num*/)
{
    node_->stopSlam();
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGKILL, signal_handler);

    rclcpp::init(argc, argv);

    node_ = std::make_shared<VSLAM>();
    rclcpp::spin(node_);
    rclcpp::shutdown();

    return 0;
}
