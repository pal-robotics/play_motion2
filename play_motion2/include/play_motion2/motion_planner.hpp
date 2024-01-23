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

#ifndef PLAY_MOTION2__MOTION_PLANNER_HPP_
#define PLAY_MOTION2__MOTION_PLANNER_HPP_

#include <map>
#include <string>
#include <vector>

#include "play_motion2/types.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace play_motion2
{
class MotionPlanner
{
public:
  MotionPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  ~MotionPlanner() = default;

  double calculate_approach_time(const MotionInfo motion_info);

  double get_reach_time(MotionPositions current_pos, MotionPositions goal_pos);

private:
  void check_parameters();
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  double approach_vel_;
  double approach_min_duration_;

  rclcpp::SubscriptionBase::SharedPtr joint_states_sub_;
  bool joint_states_updated_;
  std::map<std::string, std::vector<double>> joint_states_;
  std::mutex joint_states_mutex_;
  std::condition_variable joint_states_condition_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace play_motion2

#endif  // PLAY_MOTION2__MOTION_PLANNER_HPP_
