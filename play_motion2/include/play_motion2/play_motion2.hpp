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
#include <functional>

#include "play_motion2/motion_loader.hpp"
#include "play_motion2/motion_planner.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_msgs/srv/remove_motion.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace play_motion2
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;
using IsMotionReady = play_motion2_msgs::srv::IsMotionReady;
using ListMotions = play_motion2_msgs::srv::ListMotions;
using AddMotion = play_motion2_msgs::srv::AddMotion;
using RemoveMotion = play_motion2_msgs::srv::RemoveMotion;

using Action = play_motion2_msgs::action::PlayMotion2;
using ActionGoal = Action::Goal;
using ActionGoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using ActionResult = Action::Result;

class PlayMotion2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  PlayMotion2();
  ~PlayMotion2() override;

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
    IsMotionReady::Response::SharedPtr response);

  void get_motion_info_callback(
    GetMotionInfo::Request::ConstSharedPtr request,
    GetMotionInfo::Response::SharedPtr response) const;

  void add_motion_callback(
    AddMotion::Request::ConstSharedPtr request,
    AddMotion::Response::SharedPtr response);

  void remove_motion_callback(
    RemoveMotion::Request::ConstSharedPtr request,
    RemoveMotion::Response::SharedPtr response);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ActionGoal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ActionGoalHandle> goal_handle) const;

  void handle_accepted(const std::shared_ptr<ActionGoalHandle> goal_handle);
  void execute_motion(const std::shared_ptr<ActionGoalHandle> goal_handle);

private:
  rclcpp::Service<GetMotionInfo>::SharedPtr get_motion_info_service_;
  rclcpp::Service<IsMotionReady>::SharedPtr is_motion_ready_service_;
  rclcpp::Service<ListMotions>::SharedPtr list_motions_service_;

  rclcpp::Service<AddMotion>::SharedPtr add_motion_service_;
  rclcpp::Service<RemoveMotion>::SharedPtr remove_motion_service_;

  rclcpp_action::Server<Action>::SharedPtr pm2_action_;

  std::thread motion_executor_;
  std::atomic_bool is_busy_;

  std::unique_ptr<MotionLoader> motion_loader_;
  std::unique_ptr<MotionPlanner> motion_planner_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_HPP_
