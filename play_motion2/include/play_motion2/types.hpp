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

#ifndef PLAY_MOTION2__TYPES_HPP_
#define PLAY_MOTION2__TYPES_HPP_

#include <map>
#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "play_motion2_msgs/msg/motion.hpp"
#include "rclcpp/qos.hpp"

namespace
{
inline auto get_services_qos()
{
  #if RCLCPP_VERSION_MAJOR >= 20
  return rclcpp::QoS(rclcpp::ServicesQoS());
  #else
  return rmw_qos_profile_services_default;
  #endif
}
}  // namespace

namespace play_motion2
{

using MotionKeys = std::vector<std::string>;
using JointNames = std::vector<std::string>;
using MotionPositions = std::vector<double>;
using MotionTimes = std::vector<double>;
using TrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using Trajectory = std::vector<TrajectoryPoint>;

/**
 * @brief Structure to hold motion information.
 */
struct MotionInfo
{
  /**
   * @brief Unique key for the motion.
   */
  std::string key;

  // meta
  /**
   * @brief Human-readable name of the motion.
   */
  std::string name;
  /**
   * @brief Usage of the motion, e.g., "greeting", "walking", etc.
   */
  std::string usage;
  /**
   * @brief Description of the motion.
   */
  std::string description;

  /**
   * @brief Names of the joints involved in the motion.
   */
  JointNames joints;
  /**
   * @brief Times from the start of the motion for each position.
   */
  MotionTimes times;
  /**
   * @brief Positions for each joint at each time step.
   * The size of this vector is `joints.size() * times.size()`.
   */
  MotionPositions positions;
};

/**
 * @brief Map to hold multiple motions, indexed by their keys.
 */
using MotionsMap = std::map<std::string, MotionInfo>;

/**
 * @brief Result structure for motion execution.
 * It contains the state of the result and an optional error message.
 */
struct Result
{
  enum State
  {
    INVALID = 0,
    SUCCESS = 1,
    ERROR = 2,
    CANCELED = 3
  };

  /**
   * @brief The state of the result.
   * Can be one of: INVALID, SUCCESS, ERROR, CANCELED.
   */
  State state;
  /**
   * @brief Optional error message if the state is ERROR.
   */
  std::string error;

  /**
   * @brief Default constructor initializing the state to INVALID.
   */
  explicit Result(const State st = INVALID, const std::string & error_str = "")
  : state(st), error(error_str) {}
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__TYPES_HPP_
