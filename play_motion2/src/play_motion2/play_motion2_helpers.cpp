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

#include "rclcpp/logging.hpp"

namespace play_motion2
{

bool check_params(
  const NodeParametersInterfaceSharedPtr node_parameters_interface,
  const rclcpp::Logger & logger,
  const std::string & motion_key)
{
  const std::map<std::string, rclcpp::ParameterType> mandatory_motion_params = {
    {"joints", rclcpp::ParameterType::PARAMETER_STRING_ARRAY},
    {"positions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY},
    {"times_from_start", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY},
  };

  std::string full_param;
  bool valid_motion = true;
  for (const auto & [param, param_type] : mandatory_motion_params) {
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
  // Get optional meta data
  param = "motions." + motion_key + ".meta.name";
  if (node_parameters_interface->has_parameter(param)) {
    motion.name = node_parameters_interface->get_parameter(param).as_string();
  }

  param = "motions." + motion_key + ".meta.usage";
  if (node_parameters_interface->has_parameter(param)) {
    motion.usage = node_parameters_interface->get_parameter(param).as_string();
  }

  param = "motions." + motion_key + ".meta.description";
  if (node_parameters_interface->has_parameter(param)) {
    motion.description = node_parameters_interface->get_parameter(param).as_string();
  }

  // Get info
  param = "motions." + motion_key + ".joints";
  motion.joints = node_parameters_interface->get_parameter(param).as_string_array();

  param = "motions." + motion_key + ".positions";
  motion.positions = node_parameters_interface->get_parameter(param).as_double_array();

  param = "motions." + motion_key + ".times_from_start";
  motion.times = node_parameters_interface->get_parameter(param).as_double_array();

  // check correct size
  if (motion.positions.size() != motion.times.size() * motion.joints.size()) {
    RCLCPP_ERROR_STREAM(
      logger,
      "Motion '" << motion_key <<
        "' is not valid: sizes are not compatible. "
        "'positions' != 'joints' * 'times_from_start' (" << motion.positions.size() <<
        " != " << motion.times.size() << "*" << motion.joints.size() << ")");
    return false;
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

  for (const auto & key : all_motion_keys) {
    MotionInfo motion;
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
