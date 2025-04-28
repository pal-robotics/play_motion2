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

#ifndef UTILS__MOTION_PLANNER_HPP_
#define UTILS__MOTION_PLANNER_HPP_

#include <list>
#include <map>
#include <string>
#include <vector>

#include "play_motion2/types.hpp"

#if MOVEIT_VERSION_MINOR < 7
#include "moveit/move_group_interface/move_group_interface.h"
#else
#include "moveit_ros_planning_interface/moveit/move_group_interface/move_group_interface.hpp"
#endif

#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"

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

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using MoveGroupInterfacePtr = moveit::planning_interface::MoveGroupInterfacePtr;

class MotionPlanner
{
public:
  explicit MotionPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  ~MotionPlanner() = default;

  bool is_executable(const MotionInfo & info, const bool skip_planning);
  bool is_executable(const JointNames & joints, const bool skip_planning);

  Result execute_motion(const MotionInfo & info, const bool skip_planning);

  void cancel_motion();

private:
  void check_parameters();

  MotionInfo prepare_approach(const MotionInfo & info);

  Result perform_motion(
    const MotionInfo & info,
    const JointTrajectory & planned_approach);

  double calculate_approach_time(const MotionPositions & goal_pos, const JointNames & joints);
  double get_reach_time(MotionPositions current_pos, MotionPositions goal_pos) const;

  ControllerTrajectories generate_controller_trajectories(
    const MotionInfo & info,
    const JointTrajectory & planned_approach) const;

  JointTrajectory create_trajectory(
    const ControllerState & controller_state,
    const MotionInfo & info,
    const JointTrajectory & planned_approach) const;

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  ControllerStates get_controller_states() const;
  ControllerStates filter_controller_states(
    const ControllerStates & controller_states, const std::string & state,
    const std::string & type) const;

  bool update_controller_states_cache();

  FollowJTGoalHandleFutureResult send_trajectory(
    const std::string & controller_name,
    const JointTrajectory & trajectory);

  Result send_trajectories(
    const std::string & motion_key,
    const ControllerTrajectories & ctrl_trajectories,
    std::list<FollowJTGoalHandleFutureResult> & futures_list);

  Result wait_for_results(
    const std::vector<std::string> & motion_controllers,
    const double motion_time,
    std::list<FollowJTGoalHandleFutureResult> & futures_list);

  std::vector<MoveGroupInterfacePtr> get_valid_move_groups(const JointNames & joints) const;
  JointNames get_planned_joints(const JointNames & joints) const;

  MoveGroupInterface::Plan plan_approach(
    MoveGroupInterfacePtr group,
    const MotionInfo & approach_info);

  bool are_all_joints_included(
    const JointNames & full_joint_names,
    const JointNames & partial_joint_names) const;

  bool needs_approach(const MotionInfo & approach_info);

  void cancel_all_goals();

private:
  double approach_vel_;
  double approach_min_duration_;
  double joint_tolerance_;
  JointNames no_planning_joints_;
  std::vector<std::string> planning_groups_;

  std::vector<MoveGroupInterfacePtr> move_groups_;

  bool planning_disabled_;

  std::atomic_bool is_canceling_;

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
  rclcpp::Node::SharedPtr move_group_node_;
};

}  // namespace play_motion2

#endif  // UTILS__MOTION_PLANNER_HPP_
