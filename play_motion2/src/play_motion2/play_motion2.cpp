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

#include <stdexcept>

#include "play_motion2/motion_loader.hpp"
#include "play_motion2/motion_planner.hpp"
#include "play_motion2/play_motion2.hpp"
#include "play_motion2/types.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace play_motion2
{
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

PlayMotion2::PlayMotion2()
: LifecycleNode("play_motion2",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
  , is_motion_ready_service_()
  , list_motions_service_()
  , pm2_action_()
  , motion_executor_()
  , is_busy_(false)
{
}

PlayMotion2::~PlayMotion2()
{
  // wait if a motion is being executed until it finishes
  if (motion_executor_.joinable()) {
    motion_executor_.join();
  }
}

CallbackReturn PlayMotion2::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  motion_loader_ = std::make_unique<MotionLoader>(get_logger(), get_node_parameters_interface());
  const bool ok = motion_loader_->parse_motions();

  motion_planner_ = std::make_unique<MotionPlanner>(shared_from_this());

  RCLCPP_ERROR_EXPRESSION(get_logger(), !ok, "Failed to initialize Play Motion 2");

  return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn PlayMotion2::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  list_motions_service_ = create_service<ListMotions>(
    "play_motion2/list_motions",
    std::bind(&PlayMotion2::list_motions_callback, this, _1, _2));

  is_motion_ready_service_ = create_service<IsMotionReady>(
    "play_motion2/is_motion_ready",
    std::bind(&PlayMotion2::is_motion_ready_callback, this, _1, _2));

  pm2_action_ = rclcpp_action::create_server<Action>(
    shared_from_this(), "play_motion2",
    std::bind(&PlayMotion2::handle_goal, this, _1, _2),
    std::bind(&PlayMotion2::handle_cancel, this, _1),
    std::bind(&PlayMotion2::handle_accepted, this, _1)
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo reject when a motion is being executed ?

  list_motions_service_.reset();
  is_motion_ready_service_.reset();
  pm2_action_.reset();
  is_busy_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  motion_loader_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo cancel all goals
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void PlayMotion2::list_motions_callback(
  ListMotions::Request::ConstSharedPtr /*request*/,
  ListMotions::Response::SharedPtr response) const
{
  response->motion_keys = motion_loader_->get_motion_keys();
}

void PlayMotion2::is_motion_ready_callback(
  IsMotionReady::Request::ConstSharedPtr request,
  IsMotionReady::Response::SharedPtr response)
{
  // skip_planning argument is set to true to avoid false negatives in case planning is not enabled
  response->is_ready = !is_busy_ &&
    motion_planner_->is_executable(motion_loader_->get_motion_info(request->motion_key), true);
}

rclcpp_action::GoalResponse PlayMotion2::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ActionGoal> goal)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received goal request: motion '" << goal->motion_name << "'");

  if (is_busy_ || !motion_loader_->exists(goal->motion_name) ||
    !motion_planner_->is_executable(
      motion_loader_->get_motion_info(goal->motion_name),
      goal->skip_planning))
  {
    RCLCPP_ERROR_EXPRESSION(get_logger(), is_busy_, "PlayMotion2 is busy");
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion_name << "' cannot be performed");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (motion_executor_.joinable()) {
    motion_executor_.join();
  }
  is_busy_ = true;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlayMotion2::handle_cancel(
  const std::shared_ptr<ActionGoalHandle> goal_handle) const
{
  RCLCPP_INFO_STREAM(get_logger(), "Cancelling motion " << goal_handle->get_goal()->motion_name);
  motion_planner_->cancel_motion();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlayMotion2::handle_accepted(const std::shared_ptr<ActionGoalHandle> goal_handle)
{
  motion_executor_ = std::thread{std::bind(&PlayMotion2::execute_motion, this, _1), goal_handle};
}

void PlayMotion2::execute_motion(const std::shared_ptr<ActionGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto & motion = motion_loader_->get_motion_info(goal->motion_name);

  // Execute motion
  const auto motion_result = motion_planner_->execute_motion(motion, goal->skip_planning);

  // Evaluate and set result
  auto action_result = std::make_shared<ActionResult>();
  action_result->success = motion_result.state == Result::State::SUCCESS;
  action_result->error = motion_result.error;

  switch (motion_result.state) {
    case Result::State::SUCCESS:
      RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion_name << "' completed");
      goal_handle->succeed(action_result);
      break;
    case Result::State::ERROR:
      RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion_name << "' failed");
      goal_handle->abort(action_result);
      break;
    case Result::State::CANCELED:
      RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion_name << "' canceled");
      goal_handle->canceled(action_result);
      break;
    default:
      throw std::runtime_error("Unknown motion result state");
  }

  is_busy_ = false;
}

}  // namespace play_motion2
