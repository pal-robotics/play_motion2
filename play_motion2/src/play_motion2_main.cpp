// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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

#include "rclcpp/executors.hpp"

#include "play_motion2/play_motion2_executor.hpp"
#include "play_motion2/play_motion2_mgr.hpp"

#include "utils/motion_loader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options =
    rclcpp::NodeOptions().allow_undeclared_parameters(true).
    automatically_declare_parameters_from_overrides(
    true);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto mgrNode = std::make_shared<play_motion2::PlayMotion2Mgr>(options);
  executor.add_node(mgrNode->get_node_base_interface());

  auto executorNode = std::make_shared<play_motion2::PlayMotion2Executor>(options);
  executor.add_node(executorNode->get_node_base_interface());

  executorNode->configure();
  mgrNode->configure();
  executor.spin_some();

  executorNode->activate();
  mgrNode->activate();
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
