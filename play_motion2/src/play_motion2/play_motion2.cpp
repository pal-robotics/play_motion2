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

#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "play_motion2/play_motion2.hpp"
#include "play_motion2/play_motion2_helpers.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace play_motion2
{
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

using JTPointMsg = trajectory_msgs::msg::JointTrajectoryPoint;
using FollowJT = control_msgs::action::FollowJointTrajectory;

PlayMotion2::PlayMotion2()
: LifecycleNode("play_motion2",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
  motion_keys_({}),
  motions_({}),
  client_node_(),
  list_motions_service_(),
  pm2_action_(),
  list_controllers_client_(),
  is_motion_ready_service_()
{
}

CallbackReturn PlayMotion2::on_configure(const rclcpp_lifecycle::State & state)
{
  const bool ok =
    parse_motions(shared_from_this(), motion_keys_, motions_);

  RCLCPP_ERROR_EXPRESSION(get_logger(), !ok, "Failed to initialize Play Motion 2");

  return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn PlayMotion2::on_activate(const rclcpp_lifecycle::State & state)
{
  list_motions_service_ = create_service<ListMotions>(
    "play_motion2/list_motions",
    std::bind(&PlayMotion2::list_motions_callback, this, _1, _2));

  is_motion_ready_service_ = create_service<IsMotionReady>(
    "play_motion2/is_motion_ready",
    std::bind(&PlayMotion2::is_motion_ready_callback, this, _1, _2));

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
  is_motion_ready_service_.reset();
  pm2_action_.reset();
  client_node_.reset();
  list_controllers_client_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_cleanup(const rclcpp_lifecycle::State & state)
{
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
  ListMotions::Response::SharedPtr response) const
{
  response->motion_keys = motion_keys_;
}

void PlayMotion2::is_motion_ready_callback(
  IsMotionReady::Request::ConstSharedPtr request,
  IsMotionReady::Response::SharedPtr response) const
{
  response->is_ready = is_executable(request->motion_key);
}

rclcpp_action::GoalResponse PlayMotion2::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlayMotion2Action::Goal> goal) const
{
  RCLCPP_INFO_STREAM(get_logger(), "Received goal request: motion '" << goal->motion_name << "'");

  if (!is_executable(goal->motion_name)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion_name << "' cannot be performed");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlayMotion2::handle_cancel(
  const std::shared_ptr<GoalHandlePM2> goal_handle) const
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlayMotion2::handle_accepted(const std::shared_ptr<GoalHandlePM2> goal_handle) const
{
  std::thread{std::bind(&PlayMotion2::execute_motion, this, _1), goal_handle}.detach();
}

void PlayMotion2::execute_motion(const std::shared_ptr<GoalHandlePM2> goal_handle) const
{
  auto feedback = std::make_shared<PlayMotion2Action::Feedback>();
  auto result = std::make_shared<PlayMotion2Action::Result>();
  auto goal = goal_handle->get_goal();

  const auto ctrl_trajectories = generate_controller_trajectories(goal->motion_name);

  for (const auto & [controller, trajectory] : ctrl_trajectories) {
    if (!send_trajectory(controller, trajectory)) {
      result->success = false;
      RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion_name << "' failed");
      goal_handle->abort(result);
      return;
    }
  }

  /// @todo wait for the result to check motion is correctly completed
  /// instead of succeed.
  result->success = true;
  RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion_name << "' completed");

  goal_handle->succeed(result);
}

bool PlayMotion2::is_executable(const std::string & motion_key) const
{
  const bool is_executable = exists(motion_key) &&
    check_joints_and_controllers(motion_key);

  return is_executable;
}

bool PlayMotion2::exists(const std::string & motion_key) const
{
  const bool exists =
    std::find(motion_keys_.begin(), motion_keys_.end(), motion_key) != motion_keys_.end();

  RCLCPP_ERROR_STREAM_EXPRESSION(
    get_logger(), !exists,
    "Motion '" << motion_key << "' is not known");

  return exists;
}

ControllerStates PlayMotion2::get_controller_states() const
{
  if (!list_controllers_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Service " << list_controllers_client_->get_service_name() << " not available.");
    }
    return ControllerStates();
  }

  auto list_controllers_request = std::make_shared<ListControllers::Request>();
  auto result = list_controllers_client_->async_send_request(list_controllers_request);

  if (rclcpp::spin_until_future_complete(client_node_, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Cannot obtain " << list_controllers_client_->get_service_name() << " result");
    return ControllerStates();
  }

  return result.get()->controller;
}


ControllerStates PlayMotion2::filter_controller_states(
  const ControllerStates controller_states,
  const std::string state,
  const std::string type) const
{
  ControllerStates filtered_controller_states;

  for (const auto & controller : controller_states) {
    if (controller.state == state && controller.type == type) {
      filtered_controller_states.push_back(controller);
    }
  }

  return filtered_controller_states;
}

bool PlayMotion2::check_joints_and_controllers(const std::string & motion_key) const
{
  const auto controller_states = get_controller_states();

  if (controller_states.empty()) {
    return false;
  }

  const auto jtc_active_controllers = filter_controller_states(
    controller_states, "active",
    "joint_trajectory_controller/JointTrajectoryController");

  if (jtc_active_controllers.empty()) {
    RCLCPP_ERROR(get_logger(), "There are no active JointTrajectory controllers available");
    return false;
  }

  // get joints claimed by active controllers
  std::unordered_set<std::string> joint_names;
  for (const auto & controller : jtc_active_controllers) {
    for (const auto & interface : controller.claimed_interfaces) {
      const auto joint_name = interface.substr(0, interface.find_first_of('/'));
      joint_names.insert(joint_name);
    }
  }

  bool ok = true;
  for (const auto & joint : motions_.at(motion_key).joints) {
    // check joints are claimed by any active controller
    if (joint_names.find(joint) == joint_names.end()) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Joint '" << joint << "' is not claimed by any active controller");
      ok = false;
      continue;
    }
  }

  return ok;
}

JTMsg PlayMotion2::create_trajectory(
  const ControllerState controller_state,
  const std::string motion_key) const
{
  std::unordered_set<std::string> controller_joints;
  for (const auto & interface : controller_state.claimed_interfaces) {
    std::string joint_name = interface.substr(0, interface.find_first_of('/'));
    controller_joints.insert(joint_name);
  }

  // create a map with joints positions
  const auto & motion_info = motions_.at(motion_key);
  std::map<std::string, std::vector<double>> joint_positions;
  for (const std::string & joint : controller_joints) {
    const auto iterator = std::find(motion_info.joints.begin(), motion_info.joints.end(), joint);
    if (iterator != motion_info.joints.end()) {
      // get the location of the first position
      unsigned int vector_pos = std::distance(motion_info.joints.begin(), iterator);
      std::vector<double> positions;

      // extract positions for a specific joint and save them
      for (unsigned int i = 0; i < motion_info.times.size(); i++) {
        positions.push_back(motion_info.positions.at(vector_pos));
        vector_pos += motion_info.joints.size();
      }
      joint_positions[joint] = positions;
    }
  }

  JTMsg jt_msg;
  for (unsigned int i = 0; i < motion_info.times.size(); i++) {
    JTPointMsg jtc_point;
    const auto jtc_point_time = rclcpp::Duration::from_seconds(motion_info.times[i]);
    jtc_point.time_from_start.sec = jtc_point_time.to_rmw_time().sec;
    jtc_point.time_from_start.nanosec = jtc_point_time.to_rmw_time().nsec;

    for (const auto & j_pos : joint_positions) {
      jtc_point.positions.push_back(j_pos.second.at(i));
    }
    jt_msg.points.emplace_back(jtc_point);
  }

  for (const auto & j_pos : joint_positions) {
    jt_msg.joint_names.push_back(j_pos.first);
  }

  jt_msg.header.stamp = now();
  return jt_msg;
}

ControllerTrajectories PlayMotion2::generate_controller_trajectories(const std::string motion_key)
const
{
  const auto jtc_active_controllers = filter_controller_states(
    get_controller_states(), "active",
    "joint_trajectory_controller/JointTrajectoryController");

  ControllerTrajectories ct;
  for (const auto & controller : jtc_active_controllers) {
    const auto trajectory = create_trajectory(controller, motion_key);
    if (!trajectory.joint_names.empty()) {
      ct[controller.name] = trajectory;
    }
  }
  return ct;
}

bool PlayMotion2::send_trajectory(const std::string controller, const JTMsg trajectory) const
{
  const auto action_client = rclcpp_action::create_client<FollowJT>(
    client_node_,
    "/" + controller +
    "/follow_joint_trajectory");

  if (!action_client->wait_for_action_server(1s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return false;
  }

  /// @todo fill rest of the fields ??
  auto goal = FollowJT::Goal();
  goal.trajectory = trajectory;

  auto goal_handle = action_client->async_send_goal(goal);

  if (rclcpp::spin_until_future_complete(
      client_node_,
      goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot send goal");
    return false;
  }

  return true;
}

}  // namespace play_motion2
