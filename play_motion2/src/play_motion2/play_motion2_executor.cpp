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

#include "play_motion2_executor.hpp"

#include <utility>

#include "../utils/motion_planner.hpp"
#include "play_motion2/types.hpp"

namespace play_motion2
{
using std::placeholders::_1;
using std::placeholders::_2;

std::pair<MotionInfo, bool> convertMsg(const play_motion2_msgs::msg::Motion & msg)
{
  MotionInfo motion_info;
  bool error = false;

  if (!msg.joints.empty() &&
    !msg.positions.empty() &&
    !msg.times_from_start.empty() &&
    msg.joints.size() == msg.positions.size() / msg.times_from_start.size())
  {
    motion_info.key = msg.key;
    motion_info.name = msg.name;
    motion_info.usage = msg.usage;
    motion_info.description = msg.description;
    motion_info.joints = msg.joints;
    motion_info.positions = msg.positions;
    motion_info.times = msg.times_from_start;
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("play_motion2"),
      "Motion '" << msg.key << "' is not valid. "
                 << "Joints: " << msg.joints.size() << ", "
                 << "Positions: " << msg.positions.size() << ", "
                 << "Times: " << msg.times_from_start.size());
    error = true;
  }

  return std::make_pair(motion_info, error);
}

PlayMotion2Executor::PlayMotion2Executor(const rclcpp::NodeOptions & options)
: LifecycleNode("play_motion2_executor",
    rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
  , is_joint_list_ready_service_()
  , pm2_action_()
  , motion_executor_()
  , is_busy_(false)
{
  srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  as_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

PlayMotion2Executor::~PlayMotion2Executor()
{
  // wait if a motion is being executed until it finishes
  if (motion_executor_.joinable()) {
    motion_executor_.join();
  }
}

CallbackReturn PlayMotion2Executor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  motion_planner_ = std::make_unique<MotionPlanner>(shared_from_this());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Executor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  is_joint_list_ready_service_ = create_service<IsJointListReady>(
    "play_motion2/is_joint_list_ready",
    std::bind(&PlayMotion2Executor::isJointListReadyCb, this, _1, _2),
    get_services_qos(), srv_group_);

  pm2_action_ = rclcpp_action::create_server<Action>(
    shared_from_this(), "play_motion2/raw",
    std::bind(&PlayMotion2Executor::handleGoal, this, _1, _2),
    std::bind(&PlayMotion2Executor::handleCancel, this, _1),
    std::bind(&PlayMotion2Executor::handleAccepted, this, _1),
    rcl_action_server_get_default_options(),
    as_group_
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Executor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo reject when a motion is being executed ?

  pm2_action_.reset();
  is_busy_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Executor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Executor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo cancel all goals
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Executor::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void PlayMotion2Executor::isJointListReadyCb(
  IsJointListReady::Request::ConstSharedPtr request,
  IsJointListReady::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Received request to check joint list");
  // skip_planning argument is set to true to avoid false negatives in case planning is not enabled
  response->is_ready = !is_busy_ && motion_planner_->is_executable(request->joints, true);
  RCLCPP_INFO_STREAM(
    get_logger(), "Decided that " << (response->is_ready ? "" : "not ") <<
      "all joints are ready");
}

rclcpp_action::GoalResponse PlayMotion2Executor::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ActionGoal> goal)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received goal request: motion '" << goal->motion.key << "'");

  const auto motion = convertMsg(goal->motion);
  if (is_busy_ || motion.second ||
    !motion_planner_->is_executable(
      motion.first,
      goal->skip_planning))
  {
    RCLCPP_ERROR_EXPRESSION(get_logger(), is_busy_, "PlayMotion2 is busy");
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion.key << "' cannot be performed");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (motion_executor_.joinable()) {
    motion_executor_.join();
  }
  is_busy_ = true;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlayMotion2Executor::handleCancel(
  const std::shared_ptr<ActionGoalHandle> goal_handle) const
{
  RCLCPP_INFO_STREAM(get_logger(), "Cancelling motion " << goal_handle->get_goal()->motion.key);
  motion_planner_->cancel_motion();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlayMotion2Executor::handleAccepted(const std::shared_ptr<ActionGoalHandle> goal_handle)
{
  motion_executor_ =
    std::thread{std::bind(&PlayMotion2Executor::executeMotion, this, _1), goal_handle};
}

void PlayMotion2Executor::executeMotion(const std::shared_ptr<ActionGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto motion = convertMsg(goal->motion);

  if (motion.second) {
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion.key << "' is not valid");
    auto action_result = std::make_shared<ActionResult>();
    action_result->success = false;
    action_result->error = "Motion is not valid";
    goal_handle->abort(action_result);
    is_busy_ = false;
    return;
  }

  // Execute motion
  const auto motion_result = motion_planner_->execute_motion(motion.first, goal->skip_planning);

  // Evaluate and set result
  auto action_result = std::make_shared<ActionResult>();
  action_result->success = motion_result.state == Result::State::SUCCESS;
  action_result->error = motion_result.error;

  switch (motion_result.state) {
    case Result::State::SUCCESS:
      RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion.key << "' completed");
      goal_handle->succeed(action_result);
      break;
    case Result::State::ERROR:
      RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion.key << "' failed");
      goal_handle->abort(action_result);
      break;
    case Result::State::CANCELED:
      RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion.key << "' canceled");
      goal_handle->canceled(action_result);
      break;
    default:
      throw std::runtime_error("Unknown motion result state");
  }

  is_busy_ = false;
}
}  // namespace play_motion2
