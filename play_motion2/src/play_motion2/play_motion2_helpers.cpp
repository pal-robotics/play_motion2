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

bool parse_controllers(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  ControllerList & controllers)
{
  controllers.clear();

  if (!node_parameters_interface->has_parameter("controllers")) {
    RCLCPP_ERROR(logger, "Controllers are not defined in configuration file.");
    return false;
  }

  if (node_parameters_interface->get_parameter_types({"controllers"})[0] !=
    rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
  {
    RCLCPP_ERROR(logger, "Parameter controllers has a wrong type.");
  }

  controllers = node_parameters_interface->get_parameter("controllers").as_string_array();

  if (controllers.empty()) {
    RCLCPP_ERROR(
      logger,
      "Controllers not defined, parameter is empty.");
    return false;
  }

  return true;
}

bool check_params(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  const std::string & motion_key)
{
  const std::map<std::string, rclcpp::ParameterType> motion_params = {
    {"meta.name", rclcpp::ParameterType::PARAMETER_STRING},
    {"meta.description", rclcpp::ParameterType::PARAMETER_STRING},
    {"meta.usage", rclcpp::ParameterType::PARAMETER_STRING},
    {"joints", rclcpp::ParameterType::PARAMETER_STRING_ARRAY},
    {"positions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY},
    {"times_from_start", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY},
  };

  std::string full_param;
  bool valid_motion = true;
  for (const auto & [param, param_type] : motion_params) {
    full_param = "motions." + motion_key + "." + param;
    if (!node_parameters_interface->has_parameter(full_param)) {
      RCLCPP_ERROR_STREAM(
        logger,
        "Motion '" << motion_key << "' is not valid: parameter '" << param <<
          "' is not defined.");
      valid_motion = false;
    } else if (node_parameters_interface->get_parameter_types({full_param})[0] != param_type) {
      // check parameter type, get_parameter_types always return one param here
      RCLCPP_ERROR_STREAM(
        logger,
        "Motion '" << motion_key << "' is not valid: parameter '" << param <<
          "' has a wrong type.");
      valid_motion = false;
    }
  }
  return valid_motion;
}

MotionKeys parse_motion_keys(const NodeParametersInterfaceSharedPtr node_parameters_interface)
{
  MotionKeys motion_keys;
  std::string motion_key = "";

  // list parameters with the prefix "motions" and with any depth
  constexpr unsigned int ANY_DEPTH = 0;
  const auto params = node_parameters_interface->list_parameters({"motions"}, ANY_DEPTH);

  const int init_position = std::string("motions.").size();
  std::unordered_set<std::string> unique_names;
  for (const auto & param : params.names) {
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

bool parse_motion_info(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  const std::string & motion_key,
  MotionInfo & motion)
{
  if (!check_params(node_parameters_interface, logger, motion_key)) {
    return false;
  }

  std::string param;
  // Get meta data
  param = "motions." + motion_key + ".meta.name";
  motion.name = node_parameters_interface->get_parameter(param).as_string();

  param = "motions." + motion_key + ".meta.usage";
  motion.usage = node_parameters_interface->get_parameter(param).as_string();

  param = "motions." + motion_key + ".meta.description";
  motion.description = node_parameters_interface->get_parameter(param).as_string();

  // Get joint names
  param = "motions." + motion_key + ".joints";
  motion.joints = node_parameters_interface->get_parameter(param).as_string_array();

  // Get trajectory
  bool valid = parse_motion_trajectory(node_parameters_interface, logger, motion_key, motion);

  return valid;
}

bool parse_motion_trajectory(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  const std::string & motion_key,
  MotionInfo & motion)
{
  const std::string positions_param = "motions." + motion_key + ".positions";
  const std::string times_param = "motions." + motion_key + ".times_from_start";

  const auto joint_positions =
    node_parameters_interface->get_parameter(positions_param).as_double_array();
  const auto times_from_start =
    node_parameters_interface->get_parameter(times_param).as_double_array();
  const size_t joints_size = motion.joints.size();

  // check correct size
  if (joint_positions.size() != times_from_start.size() * joints_size) {
    RCLCPP_ERROR_STREAM(
      logger,
      "Motion '" << motion_key <<
        "' is not valid: 'positions', 'joints' and 'times_from_start' sizes are not compatible (" <<
        joint_positions.size() << " != " << times_from_start.size() << "*" << joints_size << ")");
    return false;
  }

  motion.trajectory.joint_names = motion.joints;

  auto joint_init = joint_positions.cbegin();
  for (unsigned int i = 0; i < times_from_start.size(); i++) {
    trajectory_msgs::msg::JointTrajectoryPoint jtc_point;
    jtc_point.positions.resize(joints_size);
    std::copy_n(joint_init, joints_size, jtc_point.positions.begin());

    const auto jtc_point_time = rclcpp::Duration::from_seconds(times_from_start[i]);
    jtc_point.time_from_start.sec = jtc_point_time.to_rmw_time().sec;
    jtc_point.time_from_start.nanosec = jtc_point_time.to_rmw_time().nsec;

    motion.trajectory.points.push_back(jtc_point);
    joint_init += joints_size;
  }
  return true;
}

bool parse_motions(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  MotionKeys & motion_keys,
  MotionsMap & motions)
{
  const MotionKeys all_motion_keys = parse_motion_keys(node_parameters_interface);

  motions.clear();
  motion_keys.clear();

  MotionInfo motion;
  for (const auto & key : all_motion_keys) {
    if (parse_motion_info(node_parameters_interface, logger, key, motion)) {
      motion_keys.emplace_back(key);
      motions[key] = motion;
    }
  }

  if (motion_keys.empty()) {
    RCLCPP_ERROR(
      logger,
      "No valid motions defined in configuration file.");
    return false;
  }
  return true;
}

}  // namespace play_motion2
