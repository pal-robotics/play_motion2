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

#ifndef PLAY_MOTION2__PLAY_MOTION2_HPP_
#define PLAY_MOTION2__PLAY_MOTION2_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "play_motion2/play_motion2_helpers.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace play_motion2
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using IsMotionReady = play_motion2_msgs::srv::IsMotionReady;
using ListMotions = play_motion2_msgs::srv::ListMotions;

using ListControllers = controller_manager_msgs::srv::ListControllers;
using ControllerState = controller_manager_msgs::msg::ControllerState;
using ControllerStates = std::vector<ControllerState>;

using JTMsg = trajectory_msgs::msg::JointTrajectory;
using ControllerTrajectories = std::map<std::string, JTMsg>;

using PlayMotion2Action = play_motion2_msgs::action::PlayMotion2;
using GoalHandlePM2 = rclcpp_action::ServerGoalHandle<PlayMotion2Action>;

class PlayMotion2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  PlayMotion2();
  ~PlayMotion2() = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  void list_motions_callback(
    ListMotions::Request::ConstSharedPtr request,
    ListMotions::Response::SharedPtr response) const;

  void is_motion_ready_callback(
    IsMotionReady::Request::ConstSharedPtr request,
    IsMotionReady::Response::SharedPtr response) const;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlayMotion2Action::Goal> goal) const;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePM2> goal_handle) const;

  void handle_accepted(const std::shared_ptr<GoalHandlePM2> goal_handle) const;
  void execute_motion(const std::shared_ptr<GoalHandlePM2> goal_handle) const;

  bool is_executable(const std::string & motion_key) const;

  bool exists(const std::string & motion_key) const;

  ControllerStates get_controller_states() const;
  ControllerStates filter_controller_states(
    const ControllerStates controller_states, const std::string state,
    const std::string type) const;

  bool check_joints_and_controllers(const std::string & motion_key) const;

  JTMsg create_trajectory(
    const ControllerState controller_state,
    const std::string motion_key) const;
  ControllerTrajectories generate_controller_trajectories(const std::string motion_key) const;

private:
  MotionKeys motion_keys_;
  MotionsMap motions_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Service<IsMotionReady>::SharedPtr is_motion_ready_service_;
  rclcpp::Service<ListMotions>::SharedPtr list_motions_service_;
  rclcpp_action::Server<PlayMotion2Action>::SharedPtr pm2_action_;

  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_HPP_
