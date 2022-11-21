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

#ifndef PLAY_MOTION2__PLAY_MOTION2_HELPERS_HPP_
#define PLAY_MOTION2__PLAY_MOTION2_HELPERS_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace play_motion2
{

using ControllerList = std::vector<std::string>;
using MotionKeys = std::vector<std::string>;
using MotionJoints = std::vector<std::string>;
using Trajectory = trajectory_msgs::msg::JointTrajectory;

struct MotionInfo
{
  // meta
  std::string name;
  std::string usage;
  std::string description;

  // info
  MotionJoints joints;
  Trajectory trajectory;
};

using MotionsMap = std::map<std::string, MotionInfo>;

bool parse_controllers(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  ControllerList & controllers);

// methods to parse motions
bool check_params(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & motion_key);

MotionKeys parse_motion_keys(const rclcpp_lifecycle::LifecycleNode::SharedPtr node);

bool parse_motion_info(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & motion_key,
  MotionInfo & motion);

bool parse_motion_trajectory(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & motion_key,
  MotionInfo & motion);

bool parse_motions(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  MotionKeys & motion_keys,
  MotionsMap & motions);

}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_HELPERS_HPP_
