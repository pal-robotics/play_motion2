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

#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "play_motion2/approach_planner.hpp"
#include "play_motion2/motion_loader.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
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

using FollowJT = control_msgs::action::FollowJointTrajectory;
using FollowJTGoalHandleFutureResult =
  std::shared_future<rclcpp_action::ClientGoalHandle<FollowJT>::WrappedResult>;

using PlayMotion2Action = play_motion2_msgs::action::PlayMotion2;
using GoalHandlePM2 = rclcpp_action::ServerGoalHandle<PlayMotion2Action>;

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

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlayMotion2Action::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePM2> goal_handle) const;

  void handle_accepted(const std::shared_ptr<GoalHandlePM2> goal_handle);
  void execute_motion(const std::shared_ptr<GoalHandlePM2> goal_handle);

  bool update_controller_states_cache();

  /// Check if the motion is executable using the saved cache for controller states.
  /// Be sure that the function update_controller_states_cache() is called before to update them
  bool is_executable(const std::string & motion_key) const;

  ControllerStates get_controller_states() const;
  ControllerStates filter_controller_states(
    const ControllerStates & controller_states, const std::string & state,
    const std::string & type) const;

  bool check_joints_and_controllers(const std::string & motion_key) const;

  JTMsg create_trajectory(
    const ControllerState & controller_state,
    const std::string & motion_key,
    const double approach_time) const;
  ControllerTrajectories generate_controller_trajectories(
    const std::string & motion_key,
    const double approach_time) const;

  FollowJTGoalHandleFutureResult send_trajectory(
    const std::string & controller_name,
    const JTMsg & trajectory);
  PlayMotion2Action::Result::SharedPtr wait_for_results(
    const std::shared_ptr<GoalHandlePM2> goal_handle,
    std::list<FollowJTGoalHandleFutureResult> & futures_list,
    const double motion_time);

private:
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Service<IsMotionReady>::SharedPtr is_motion_ready_service_;
  rclcpp::Service<ListMotions>::SharedPtr list_motions_service_;
  rclcpp_action::Server<PlayMotion2Action>::SharedPtr pm2_action_;

  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;

  std::map<std::string, rclcpp_action::Client<FollowJT>::SharedPtr> action_clients_;

  ControllerStates motion_controller_states_;

  std::thread motion_executor_;
  std::atomic_bool is_busy_;

  std::unique_ptr<MotionLoader> motion_loader_;
  std::unique_ptr<ApproachPlanner> approach_planner_;
  std::unique_ptr<std::thread> spinner_thread_;
  rclcpp::executors::MultiThreadedExecutor executor_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_HPP_
