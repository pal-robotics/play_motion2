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

#include "play_motion2/play_motion2.hpp"
#include "play_motion2/play_motion2_helpers.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace play_motion2
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

PlayMotion2::PlayMotion2()
: LifecycleNode("play_motion2",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
  motion_keys_({}),
  motions_({}),
  client_node_(nullptr),
  list_motions_service_(nullptr),
  pm2_action_(nullptr),
  list_controllers_client_(nullptr)
{
}

CallbackReturn PlayMotion2::on_configure(const rclcpp_lifecycle::State & state)
{
  const bool ok =
    parse_controllers(shared_from_this(), controllers_) &&
    parse_motions(shared_from_this(), motion_keys_, motions_);

  RCLCPP_ERROR_EXPRESSION(get_logger(), !ok, "Failed to initialize Play Motion 2");

  return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn PlayMotion2::on_activate(const rclcpp_lifecycle::State & state)
{
  list_motions_service_ = create_service<ListMotions>(
    "play_motion2/list_motions",
    std::bind(&PlayMotion2::list_motions_callback, this, _1, _2));

  pm2_action_ = rclcpp_action::create_server<PlayMotion2Action>(
    shared_from_this(), "play_motion2",
    std::bind(&PlayMotion2::handle_goal, this, _1, _2),
    std::bind(&PlayMotion2::handle_cancel, this, _1),
    std::bind(&PlayMotion2::handle_accepted, this, _1)
  );

  client_node_ = rclcpp::Node::make_shared("client_node");
  list_controllers_client_ = client_node_->create_client<ListControllers>(
    "/controller_manager/list_controllers");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_deactivate(const rclcpp_lifecycle::State & state)
{
  list_motions_service_.reset();
  pm2_action_.reset();
  client_node_.reset();
  list_controllers_client_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_cleanup(const rclcpp_lifecycle::State & state)
{
  controllers_.clear();
  motion_keys_.clear();
  motions_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_error(const rclcpp_lifecycle::State & state)
{
  return CallbackReturn::SUCCESS;
}

void PlayMotion2::list_motions_callback(
  ListMotions::Request::ConstSharedPtr /*request*/,
  ListMotions::Response::SharedPtr response)
{
  response->motion_keys = motion_keys_;
}

rclcpp_action::GoalResponse PlayMotion2::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlayMotion2Action::Goal> goal)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received goal request: motion '" << goal->motion_name << "'");

  if (!is_executable(goal->motion_name)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion_name << "' cannot be performed");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlayMotion2::handle_cancel(
  const std::shared_ptr<GoalHandlePM2> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlayMotion2::handle_accepted(const std::shared_ptr<GoalHandlePM2> goal_handle)
{
  std::thread{std::bind(&PlayMotion2::execute_motion, this, _1), goal_handle}.detach();
}

void PlayMotion2::execute_motion(const std::shared_ptr<GoalHandlePM2> goal_handle)
{
  auto feedback = std::make_shared<PlayMotion2Action::Feedback>();
  auto result = std::make_shared<PlayMotion2Action::Result>();
  auto goal = goal_handle->get_goal();

  result->success = true;
  RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion_name << "' completed");

  goal_handle->succeed(result);
}

bool PlayMotion2::is_executable(const std::string & motion_name)
{
  // check the motion is in the list
  if (std::find(motion_keys_.begin(), motion_keys_.end(), motion_name) == motion_keys_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << motion_name << "' is not known");
    return false;
  }

  // get the list of controller states available
  if (!list_controllers_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Service " << list_controllers_client_->get_service_name() << " not available.");
    }
    return false;
  }

  auto list_controllers_request = std::make_shared<ListControllers::Request>();
  auto result = list_controllers_client_->async_send_request(list_controllers_request);

  if (rclcpp::spin_until_future_complete(client_node_, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Cannot obtain " << list_controllers_client_->get_service_name() << " result");
    return false;
  }
  auto controller_states = result.get()->controller;

  // get available controllers and their claimed joints
  ControllerList available_controllers;
  std::unordered_set<std::string> available_joints;
  for (const auto & controller : controller_states) {
    if (std::find(
        controllers_.begin(), controllers_.end(),
        controller.name) != controllers_.end())
    {
      available_controllers.push_back(controller.name);
      for (const auto & interface : controller.claimed_interfaces) {
        available_joints.insert(interface.substr(0, interface.find_first_of('/')));
      }
    }
  }

  // check all controllers are available
  ControllerList controllers_diff;
  std::set_difference(
    controllers_.begin(), controllers_.end(),
    available_controllers.begin(), available_controllers.end(),
    back_inserter(controllers_diff));

  if (controllers_diff.size() > 0) {
    for (const auto & controller : controllers_diff) {
      RCLCPP_ERROR_STREAM(get_logger(), "Controller '" << controller << "' is not available");
    }
    return false;
  }

  // check joints are claimed by any controller
  for (const auto & joint : motions_[motion_name].joints) {
    if (std::find(
        available_joints.begin(), available_joints.end(),
        joint) == available_joints.end())
    {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Joint '" << joint << "' is not claimed by any specified controller");
      return false;
    }
  }
  return true;
}


}  // namespace play_motion2
