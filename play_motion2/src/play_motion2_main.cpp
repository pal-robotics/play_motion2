// Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

#include "lifecycle_msgs/msg/transition.hpp"
#include "play_motion2/play_motion2.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto play_motion2 = std::make_shared<play_motion2::PlayMotion2>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(play_motion2->get_node_base_interface());

  play_motion2->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();

  play_motion2->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
