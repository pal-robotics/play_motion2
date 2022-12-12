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

namespace play_motion2
{

using MotionKeys = std::vector<std::string>;
using MotionJoints = std::vector<std::string>;
using MotionPositions = std::vector<double>;
using MotionTimes = std::vector<double>;

using NodeParametersInterfaceSharedPtr =
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr;

struct MotionInfo
{
  // meta
  std::string name;
  std::string usage;
  std::string description;

  // info
  MotionJoints joints;
  MotionPositions positions;
  MotionTimes times;
};

using MotionsMap = std::map<std::string, MotionInfo>;

// methods to parse motions
bool check_params(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  const std::string & motion_key);

template<typename NodeT>
bool check_params(
  const NodeT & node,
  const std::string & motion_key)
{
  return check_params(node->get_node_parameters_interface(), node->get_logger(), motion_key);
}

MotionKeys parse_motion_keys(const NodeParametersInterfaceSharedPtr node_parameters_interface);

template<typename NodeT>
MotionKeys parse_motion_keys(const NodeT & node)
{
  return parse_motion_keys(node->get_node_parameters_interface());
}

bool parse_motion_info(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  const std::string & motion_key,
  MotionInfo & motion);

template<typename NodeT>
bool parse_motion_info(
  const NodeT & node,
  const std::string & motion_key,
  MotionInfo & motion)
{
  return parse_motion_info(
    node->get_node_parameters_interface(),
    node->get_logger(), motion_key, motion);
}

bool parse_motions(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  MotionKeys & motion_keys,
  MotionsMap & motions);

template<typename NodeT>
bool parse_motions(
  const NodeT & node,
  MotionKeys & motion_keys,
  MotionsMap & motions)
{
  return parse_motions(
    node->get_node_parameters_interface(),
    node->get_logger(), motion_keys, motions);
}

}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_HELPERS_HPP_
