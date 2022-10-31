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

#include "play_motion2/play_motion2.hpp"
#include "rclcpp/logging.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace play_motion2
{

// there has to be a better way to do it
rclcpp::NodeOptions get_pm_node_options()
{
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  return node_options;
}

PlayMotion2::PlayMotion2()
: Node("play_motion2", get_pm_node_options()),
  motion_keys_({}), motions_({})
{
}

void PlayMotion2::init()
{
  parse_motions(shared_from_this(), motion_keys_, motions_);
}

}  // namespace play_motion2
