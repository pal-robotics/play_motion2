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

#include "rclcpp/node.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

typedef std::vector<std::string> MotionKeys;
typedef std::vector<std::string> MotionJoints;
typedef trajectory_msgs::msg::JointTrajectory Trajectory;

namespace play_motion2
{

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

// methods to parse motions
MotionKeys parse_motion_keys(const rclcpp::Node::SharedPtr node);
MotionInfo parse_motion_info(
  const rclcpp::Node::SharedPtr node,
  const std::string & motion_key);
Trajectory parse_motion_trajectory(
  const rclcpp::Node::SharedPtr node,
  const std::string & motion_key,
  const MotionJoints & joints);
void parse_motions(
  const rclcpp::Node::SharedPtr node,
  MotionKeys & motion_keys,
  std::map<std::string, MotionInfo> & motions);

}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_HELPERS_HPP_
