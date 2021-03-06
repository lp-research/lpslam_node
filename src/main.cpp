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
#define VSLAM lpslam_components::LpSlamNode
#else
#include "OpenVSLAMNode.hpp"
#define VSLAM lpslam_components::OpenVSLAMNode
#endif  // USE_OPENVSLAM_DIRECTLY


int main(int argc, char **argv)
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);


    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;


    auto node = std::make_shared<VSLAM>(options);

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}
