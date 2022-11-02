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

#include "play_motion2/play_motion2_helpers.hpp"

namespace play_motion2
{

void parse_controllers(
  const rclcpp::Node::SharedPtr node,
  ControllerList & controllers)
{
  node->get_parameter_or("controllers", controllers, {});
}

MotionKeys parse_motion_keys(const rclcpp::Node::SharedPtr node)
{
  MotionKeys motion_keys;
  const auto params = node->list_parameters({"motions"}, 0);
  std::string motion_key = "";

  const int init_position = std::string("motions.").size();
  std::unordered_set<std::string> unique_names;
  for (auto & param : params.names) {
    // find the motion key: after 'motions.', and before the next '.'
    motion_key = param.substr(
      init_position,
      param.find_first_of('.', init_position) - init_position);
    // Add the motion to the set
    unique_names.insert(motion_key);
  }

  motion_keys.assign(unique_names.begin(), unique_names.end());
  return motion_keys;
}

MotionInfo parse_motion_info(
  const rclcpp::Node::SharedPtr node,
  const std::string & motion_key)
{
  MotionInfo motion;
  std::string param;

  // Get meta data
  param = "motions." + motion_key + ".meta.name";
  motion.name = node->get_parameter(param).as_string();

  param = "motions." + motion_key + ".meta.usage";
  motion.usage = node->get_parameter(param).as_string();

  param = "motions." + motion_key + ".meta.description";
  motion.description = node->get_parameter(param).as_string();

  // Get joint names
  param = "motions." + motion_key + ".joints";
  motion.joints = node->get_parameter(param).as_string_array();

  // Get trajectory
  motion.trajectory = parse_motion_trajectory(node, motion_key, motion.joints);

  return motion;
}

Trajectory parse_motion_trajectory(
  const rclcpp::Node::SharedPtr node,
  const std::string & motion_key,
  const MotionJoints & joints)
{
  const std::string positions_param = "motions." + motion_key + ".positions";
  const std::string times_param = "motions." + motion_key + ".times_from_start";

  const auto joint_positions = node->get_parameter(positions_param).as_double_array();
  const auto times_from_start = node->get_parameter(times_param).as_double_array();
  const int joints_size = joints.size();

  // check correct size
  if (joint_positions.size() != times_from_start.size() * joints_size) {
    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "Positions size (" << joint_positions.size() <<
        ") is not compatible with number of joints (" <<
        joints_size <<
        ") and times (" << times_from_start.size() <<
        ") for motion '" << motion_key << "'");
  }

  Trajectory trajectory;
  trajectory.joint_names = joints;

  auto joint_init = joint_positions.begin();

  for (int i = 0; i < times_from_start.size(); i++) {
    trajectory_msgs::msg::JointTrajectoryPoint jtc_point;
    jtc_point.positions.resize(joints_size);
    std::copy(joint_init, joint_init + joints_size, jtc_point.positions.begin());

    auto my_time = rclcpp::Duration::from_seconds(times_from_start[i]);
    jtc_point.time_from_start.sec = my_time.to_rmw_time().sec;
    jtc_point.time_from_start.nanosec = my_time.to_rmw_time().nsec;

    trajectory.points.push_back(jtc_point);
    joint_init += joints_size;
  }
  return trajectory;
}

void parse_motions(
  const rclcpp::Node::SharedPtr node,
  MotionKeys & motion_keys,
  std::map<std::string, MotionInfo> & motions)
{
  motion_keys = parse_motion_keys(node);

  for (auto & key : motion_keys) {
    motions[key] = parse_motion_info(node, key);
  }
}

}  // namespace play_motion2
