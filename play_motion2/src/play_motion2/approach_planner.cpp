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

#include "play_motion2/approach_planner.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace play_motion2
{
using std::placeholders::_1;

constexpr double kDefaultApproachVel = 0.5;
constexpr double kDefaultApproachMinDuration = 0.0;

ApproachPlanner::ApproachPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
:   approach_vel_(kDefaultApproachVel),
  approach_min_duration_(kDefaultApproachMinDuration),

  joint_states_sub_(nullptr),
  joint_states_updated_(false),
  joint_states_(),
  joint_states_mutex_(),
  joint_states_condition_(),

  node_(node)
{
  joint_states_sub_ =
    node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1,
    std::bind(&ApproachPlanner::joint_states_callback, this, _1));
}

void ApproachPlanner::check_parameters()
{
  const bool good_approach_vel = node_->has_parameter("approach_velocity") &&
    node_->get_parameter_types({"approach_velocity"})[0] ==
    rclcpp::ParameterType::PARAMETER_DOUBLE &&
    node_->get_parameter("approach_velocity").as_double() > 0.0;

  if (good_approach_vel) {approach_vel_ = node_->get_parameter("approach_velocity").as_double();}

  RCLCPP_WARN_STREAM_EXPRESSION(
    node_->get_logger(), !good_approach_vel,
    "Param approach_velocity not set, wrong typed, negative or 0, using the default value: " <<
      kDefaultApproachVel);

  const bool good_approach_min_duration = node_->has_parameter("approach_min_duration") &&
    node_->get_parameter_types({"approach_min_duration"})[0] ==
    rclcpp::ParameterType::PARAMETER_DOUBLE &&
    node_->get_parameter("approach_min_duration").as_double() >= 0.0;

  if (good_approach_min_duration) {
    approach_min_duration_ = node_->get_parameter("approach_min_duration").as_double();
  }

  RCLCPP_WARN_STREAM_EXPRESSION(
    node_->get_logger(), !good_approach_min_duration,
    "Param approach_min_duration not set, wrong typed or negative, using the default value: " <<
      kDefaultApproachVel);
}

double ApproachPlanner::calculate_approach_time(const MotionInfo motion_info)
{
  check_parameters();

  // first position for all joints
  MotionPositions goal_pos =
  {motion_info.positions.begin(),
    motion_info.positions.begin() + motion_info.joints.size()};

  // wait until joint_states updated and set current positions
  std::unique_lock<std::mutex> lock(joint_states_mutex_);
  joint_states_updated_ = false;
  joint_states_condition_.wait(lock, [&] {return joint_states_updated_;});

  MotionPositions curr_pos;
  for (const auto & joint : motion_info.joints) {
    curr_pos.push_back(joint_states_[joint][0]);
  }
  lock.unlock();

  return get_reach_time(curr_pos, goal_pos);
}

double ApproachPlanner::get_reach_time(MotionPositions current_pos, MotionPositions goal_pos)
{
  // Maximum joint displacement
  double dmax = 0.0;
  for (unsigned int i = 0; i < current_pos.size(); ++i) {
    const double d = std::abs(goal_pos[i] - current_pos[i]);
    if (d > dmax) {
      dmax = d;
    }
  }
  return std::max(dmax / approach_vel_, approach_min_duration_);
}

void ApproachPlanner::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(joint_states_mutex_);
  if (!joint_states_updated_) {
    joint_states_.clear();
    for (size_t i = 0; i < msg->name.size(); i++) {
      joint_states_[msg->name[i]] = {msg->position[i], msg->velocity[i], msg->effort[i]};
    }
    joint_states_updated_ = true;
  }
  joint_states_condition_.notify_one();
}

}  // namespace play_motion2
