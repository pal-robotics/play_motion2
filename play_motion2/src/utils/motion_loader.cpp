// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

#include <map>
#include <string>
#include <unordered_set>

#include "motion_loader.hpp"
#include "rclcpp/logging.hpp"

namespace play_motion2
{
MotionLoader::MotionLoader(
  const rclcpp::Logger logger,
  const NodeParametersInterfaceSharedPtr parameters_interface)
: logger_(logger)
  , params_interface_(parameters_interface)
  , motion_keys_({})
  , motions_({})
{
}

MotionKeys MotionLoader::parse_motion_keys() const
{
  MotionKeys motion_keys;
  std::string motion_key = "";

  // list parameters with the prefix "motions" and with any depth
  constexpr auto ANY_DEPTH = 0u;
  const auto params = params_interface_->list_parameters({"motions"}, ANY_DEPTH);

  const int init_position = std::string("motions.").size();
  std::unordered_set<std::string> unique_names;
  for (const auto & param : params.names) {
    // find the motion key: after 'motions.', and before the next '.'
    motion_key = param.substr(
      init_position,
      param.find_first_of('.', init_position) - init_position);
    // Add the motion to the set of unique values
    unique_names.insert(motion_key);
  }

  motion_keys.assign(unique_names.begin(), unique_names.end());
  return motion_keys;
}

bool MotionLoader::check_params(const std::string & motion_key) const
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
    if (!params_interface_->has_parameter(full_param)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Motion '" << motion_key << "' is not valid: parameter '" << param <<
          "' is not defined.");
      valid_motion = false;
    } else if (params_interface_->get_parameter_types({full_param})[0] != param_type) {
      // check parameter type, get_parameter_types always return one param here
      RCLCPP_ERROR_STREAM(
        logger_,
        "Motion '" << motion_key << "' is not valid: parameter '" << param <<
          "' has a wrong type.");
      valid_motion = false;
    }
  }
  return valid_motion;
}

bool MotionLoader::parse_motion_info(const std::string & motion_key)
{
  if (!check_params(motion_key)) {
    return false;
  }

  MotionInfo motion;
  motion.key = motion_key;

  std::string param;
  // Get optional meta data
  param = "motions." + motion_key + ".meta.name";
  if (params_interface_->has_parameter(param)) {
    motion.name = params_interface_->get_parameter(param).as_string();
  }

  param = "motions." + motion_key + ".meta.usage";
  if (params_interface_->has_parameter(param)) {
    motion.usage = params_interface_->get_parameter(param).as_string();
  }

  param = "motions." + motion_key + ".meta.description";
  if (params_interface_->has_parameter(param)) {
    motion.description = params_interface_->get_parameter(param).as_string();
  }

  // Get info
  param = "motions." + motion_key + ".joints";
  motion.joints = params_interface_->get_parameter(param).as_string_array();

  param = "motions." + motion_key + ".positions";
  motion.positions = params_interface_->get_parameter(param).as_double_array();

  param = "motions." + motion_key + ".times_from_start";
  motion.times = params_interface_->get_parameter(param).as_double_array();

  // check correct size
  if (motion.positions.size() != motion.times.size() * motion.joints.size()) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Motion '" << motion_key <<
        "' is not valid: sizes are not compatible. "
        "'positions' != 'joints' * 'times_from_start' (" << motion.positions.size() <<
        " != " << motion.times.size() << "*" << motion.joints.size() << ")");
    return false;
  }
  motion_keys_.emplace_back(motion_key);
  motions_[motion_key] = motion;
  return true;
}

bool MotionLoader::exists(const std::string & motion_key) const
{
  const bool exists =
    std::find(motion_keys_.begin(), motion_keys_.end(), motion_key) != motion_keys_.end();

  return exists;
}

bool MotionLoader::parse_motions()
{
  const MotionKeys all_motion_keys = parse_motion_keys();

  motions_.clear();
  motion_keys_.clear();

  for (const auto & key : all_motion_keys) {
    parse_motion_info(key);
  }

  if (motion_keys_.empty()) {
    RCLCPP_ERROR(
      logger_,
      "No valid motions defined in configuration file.");
    return false;
  }
  return true;
}

const MotionKeys & MotionLoader::get_motion_keys() const
{
  return motion_keys_;
}

const MotionInfo & MotionLoader::get_motion_info(const std::string & motion_key) const
{
  return motions_.at(motion_key);
}

const MotionsMap & MotionLoader::get_motions() const
{
  return motions_;
}

bool MotionLoader::add_motion(const MotionMsg & motion_msg, const bool overwrite)
{
  if (motion_msg.key.empty()) {
    RCLCPP_ERROR(logger_, "Motion key is empty.");
    return false;
  }

  if (exists(motion_msg.key) && !overwrite) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Motion '" << motion_msg.key << "' already exists and overwrite option is disabled.");
    return false;
  }

  // Check sizes
  if (motion_msg.joints.empty() || motion_msg.positions.empty() ||
    motion_msg.times_from_start.empty())
  {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Motion '" << motion_msg.key <<
        "' is not valid: empty 'joints', 'positions' or 'times_from_start'.");
    return false;
  }

  if (motion_msg.joints.size() !=
    motion_msg.positions.size() / motion_msg.times_from_start.size())
  {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Motion '" << motion_msg.key <<
        "' is not valid: sizes are not compatible. "
        "'positions' != 'joints' * 'times_from_start' (" << motion_msg.positions.size() <<
        " != " << motion_msg.times_from_start.size() << "*" << motion_msg.joints.size() << ")");
    return false;
  }

  // Create motion info from message
  MotionInfo motion_info;
  motion_info.key = motion_msg.key;
  motion_info.joints = motion_msg.joints;
  motion_info.positions = motion_msg.positions;
  motion_info.times = motion_msg.times_from_start;
  motion_info.name = motion_msg.name;
  motion_info.description = motion_msg.description;
  motion_info.usage = motion_msg.usage;

  // Add motion info and key
  if (!exists(motion_info.key)) {
    motion_keys_.emplace_back(motion_info.key);
  }
  motions_[motion_info.key] = motion_info;

  return true;
}

bool MotionLoader::remove_motion(const std::string & motion_key)
{
  if (!exists(motion_key)) {
    RCLCPP_ERROR_STREAM(logger_, "Motion '" << motion_key << "' does not exist.");
    return false;
  }

  // Remove both motion info and key
  motions_.erase(motion_key);
  motion_keys_.erase(
    std::remove(motion_keys_.begin(), motion_keys_.end(), motion_key),
    motion_keys_.end());

  return true;
}

}  // namespace play_motion2
