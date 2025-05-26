// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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

#ifndef PLAY_MOTION2__PLAY_MOTION2_EXECUTOR_HPP_
#define PLAY_MOTION2__PLAY_MOTION2_EXECUTOR_HPP_

#include <memory>

#include "play_motion2_msgs/action/play_motion2_raw.hpp"
#include "play_motion2_msgs/srv/is_joint_list_ready.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace play_motion2
{
class MotionPlanner;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using IsJointListReady = play_motion2_msgs::srv::IsJointListReady;

using Action = play_motion2_msgs::action::PlayMotion2Raw;
using ActionGoal = Action::Goal;
using ActionGoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using ActionResult = Action::Result;

/**
 * @class PlayMotion2Executor
 * @brief A lifecycle node that executes motions using the PlayMotion2 action server.
 * It provides a service to check if a list of joints is ready to receive a motion. It provides
 * an action server to handle motion execution requests as well.
 */
class PlayMotion2Executor : public rclcpp_lifecycle::LifecycleNode
{
public:
  // Callback group for the is_joint_list_ready service
  rclcpp::CallbackGroup::SharedPtr srv_group_;
  // Callback group for the action server
  rclcpp::CallbackGroup::SharedPtr as_group_;

  explicit PlayMotion2Executor(const rclcpp::NodeOptions & options);
  virtual ~PlayMotion2Executor();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  void isJointListReadyCb(
    IsJointListReady::Request::ConstSharedPtr request,
    IsJointListReady::Response::SharedPtr response);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ActionGoal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<ActionGoalHandle> goal_handle)
  const;
  void handleAccepted(const std::shared_ptr<ActionGoalHandle> goal_handle);
  void executeMotion(const std::shared_ptr<ActionGoalHandle> goal_handle);

private:
  rclcpp::Service<IsJointListReady>::SharedPtr is_joint_list_ready_service_;
  rclcpp_action::Server<Action>::SharedPtr pm2_action_;

  std::thread motion_executor_;
  std::atomic_bool is_busy_;

  std::unique_ptr<MotionPlanner> motion_planner_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_EXECUTOR_HPP_
