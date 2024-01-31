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

#include <list>
#include <map>
#include <string>
#include <vector>

#include "play_motion2/types.hpp"

#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/callback_group.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace play_motion2
{
using ControllerState = controller_manager_msgs::msg::ControllerState;
using ControllerStates = std::vector<ControllerState>;
using ListControllers = controller_manager_msgs::srv::ListControllers;

using JointState = sensor_msgs::msg::JointState;

using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using ControllerTrajectories = std::map<std::string, JointTrajectory>;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FollowJTGoalHandleFutureResult =
  std::shared_future<rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult>;

class MotionPlanner
{
public:
  explicit MotionPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  ~MotionPlanner() = default;

  bool is_executable(const MotionInfo & motion_info);

  Result execute_motion(const std::string & motion_key, const MotionInfo & motion_info);

  void cancel_motion();

private:
  void check_parameters();

  MotionInfo prepare_approach(const MotionInfo & motion_info);
  MotionInfo prepare_motion(const MotionInfo & motion_info);

  Result perform_unplanned_motion(const std::string & motion_key, const MotionInfo & motion_info);

  double calculate_approach_time(const MotionPositions & goal_pos, const JointNames & joints);
  double get_reach_time(MotionPositions current_pos, MotionPositions goal_pos) const;

  ControllerTrajectories generate_controller_trajectories(const MotionInfo & motion_info) const;

  JointTrajectory create_trajectory(
    const ControllerState & controller_state,
    const MotionInfo & motion_info,
    const double extra_time) const;

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  ControllerStates get_controller_states() const;
  ControllerStates filter_controller_states(
    const ControllerStates & controller_states, const std::string & state,
    const std::string & type) const;

  bool update_controller_states_cache();
  bool check_joints_and_controllers(const std::string & motion_key) const;

  FollowJTGoalHandleFutureResult send_trajectory(
    const std::string & controller_name,
    const JointTrajectory & trajectory);

  Result send_trajectories(
    const std::string & motion_key, const MotionInfo & motion_info,
    std::list<FollowJTGoalHandleFutureResult> & futures_list);

  Result wait_for_results(
    std::list<FollowJTGoalHandleFutureResult> & futures_list,
    const double motion_time);

private:
  // bool planning_disabled_;
  // bool planned_approach_;

  double approach_vel_;
  double approach_min_duration_;

  std::atomic_bool is_canceling_;
  bool busy_;

  ControllerStates motion_controller_states_;

  rclcpp::SubscriptionBase::SharedPtr joint_states_sub_;
  bool joint_states_updated_;
  std::map<std::string, std::vector<double>> joint_states_;
  std::mutex joint_states_mutex_;
  std::condition_variable joint_states_condition_;

  rclcpp::CallbackGroup::SharedPtr motion_planner_cb_group_;
  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;

  std::map<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> action_clients_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace play_motion2

#endif  // PLAY_MOTION2__MOTION_PLANNER_HPP_
