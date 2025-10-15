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

#include "play_motion2_mgr.hpp"

#include "../utils/motion_loader.hpp"

namespace play_motion2
{
using std::placeholders::_1;
using std::placeholders::_2;

PlayMotion2Mgr::PlayMotion2Mgr(const rclcpp::NodeOptions & options)
: LifecycleNode("play_motion2_mgr",
    rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
  , is_motion_ready_service_()
  , list_motions_service_()
  , pm2_action_()
{
  ac_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  as_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_c_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_s_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

PlayMotion2Mgr::~PlayMotion2Mgr()
{
}

CallbackReturn PlayMotion2Mgr::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  const bool ok = configureMgr();

  if (ok) {
    RCLCPP_INFO(get_logger(), "Play Motion 2 initialized");
    return CallbackReturn::SUCCESS;
  }

  RCLCPP_ERROR(get_logger(), "Failed to initialize Play Motion 2");
  return CallbackReturn::FAILURE;
}

CallbackReturn PlayMotion2Mgr::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  list_motions_service_ = create_service<ListMotions>(
    "play_motion2/list_motions",
    std::bind(&PlayMotion2Mgr::listMotionsCallback, this, _1, _2),
    get_services_qos(), srv_s_group_);

  is_motion_ready_service_ = create_service<IsMotionReady>(
    "play_motion2/is_motion_ready",
    std::bind(&PlayMotion2Mgr::isMotionReadyCallback, this, _1, _2),
    get_services_qos(), srv_s_group_);

  get_motion_info_service_ = create_service<GetMotionInfo>(
    "play_motion2/get_motion_info",
    std::bind(&PlayMotion2Mgr::getMotionInfoCallback, this, _1, _2),
    get_services_qos(), srv_s_group_);

  add_motion_service_ = create_service<AddMotion>(
    "play_motion2/add_motion",
    std::bind(&PlayMotion2Mgr::addMotionCallback, this, _1, _2),
    get_services_qos(), srv_s_group_);

  remove_motion_service_ = create_service<RemoveMotion>(
    "play_motion2/remove_motion",
    std::bind(&PlayMotion2Mgr::removeMotionCallback, this, _1, _2),
    get_services_qos(), srv_s_group_);

  pm2_action_ = rclcpp_action::create_server<play_motion2_msgs::action::PlayMotion2>(
    shared_from_this(), "play_motion2",
    std::bind(&PlayMotion2Mgr::handleGoal, this, _1, _2),
    std::bind(&PlayMotion2Mgr::handleCancel, this, _1),
    std::bind(&PlayMotion2Mgr::handleAccepted, this, _1),
    rcl_action_server_get_default_options(),
    as_group_
  );

  is_joint_list_ready_client_ = create_client<IsJointListReady>(
    "play_motion2/is_joint_list_ready",
    get_services_qos(), srv_c_group_);

  if (!is_joint_list_ready_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to is_joint_list_ready service");
    return CallbackReturn::FAILURE;
  }

  pm2_raw_action_ = rclcpp_action::create_client<play_motion2_msgs::action::PlayMotion2Raw>(
    shared_from_this(), "play_motion2/raw",
    ac_group_);
  if (!pm2_raw_action_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to play_motion2/raw action server");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Mgr::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo reject when a motion is being executed ?

  list_motions_service_.reset();
  is_motion_ready_service_.reset();
  get_motion_info_service_.reset();
  add_motion_service_.reset();
  remove_motion_service_.reset();

  pm2_action_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Mgr::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  cleanupMgr();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Mgr::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo cancel all goals
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2Mgr::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void PlayMotion2Mgr::listMotionsCallback(
  ListMotions::Request::ConstSharedPtr /*request*/,
  ListMotions::Response::SharedPtr response) const
{
  response->motion_keys = getMotionKeys();
}

void PlayMotion2Mgr::isMotionReadyCallback(
  IsMotionReady::Request::ConstSharedPtr request,
  IsMotionReady::Response::SharedPtr response)
{
  // skip_planning argument is set to true to avoid false negatives in case planning is not enabled
  if (motionExists(request->motion_key)) {
    auto motion = loadMotion(request->motion_key);
    auto is_joint_list_ready_request =
      std::make_shared<IsJointListReady::Request>();
    is_joint_list_ready_request->joints = motion.joints;

    auto is_joint_list_ready_response =
      is_joint_list_ready_client_->async_send_request(is_joint_list_ready_request);
    RCLCPP_INFO_STREAM(
      get_logger(), "Sending request to is_joint_list_ready service");

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    while (is_joint_list_ready_response.wait_for(std::chrono::milliseconds(100)) ==
      std::future_status::timeout)
    {
      if (!rclcpp::ok() ||
        (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)))
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to call is_joint_list_ready service");
        response->is_ready = false;
        return;
      }
    }

    response->is_ready = is_joint_list_ready_response.get()->is_ready;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << request->motion_key << "' does not exist");
    response->is_ready = false;
  }

  RCLCPP_INFO_STREAM(
    get_logger(), "Motion '" << request->motion_key << "' is "
                             << (response->is_ready ? "ready" : "not ready"));
}

void PlayMotion2Mgr::getMotionInfoCallback(
  GetMotionInfo::Request::ConstSharedPtr request,
  GetMotionInfo::Response::SharedPtr response) const
{
  if (!motionExists(request->motion_key)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << request->motion_key << "' does not exist");
    return;
  }

  response->motion = loadMotion(request->motion_key);
}

void PlayMotion2Mgr::addMotionCallback(
  AddMotion::Request::ConstSharedPtr request,
  AddMotion::Response::SharedPtr response)
{
  response->success = addMotion(request->motion, request->overwrite);
}

void PlayMotion2Mgr::removeMotionCallback(
  RemoveMotion::Request::ConstSharedPtr request,
  RemoveMotion::Response::SharedPtr response)
{
  response->success = removeMotion(request->motion_key);
}

rclcpp_action::GoalResponse PlayMotion2Mgr::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const play_motion2_msgs::action::PlayMotion2::Goal> goal)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received goal request: motion '" << goal->motion_name << "'");

  if (!motionExists(goal->motion_name)) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Motion '" << goal->motion_name << "' does not exist");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlayMotion2Mgr::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<play_motion2_msgs::action::PlayMotion2>>)
const
{
  // Cancel Raw action
  if (current_goal_) {
    pm2_raw_action_->async_cancel_goal(current_goal_);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlayMotion2Mgr::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    play_motion2_msgs::action::PlayMotion2>> goal_handle)
{
  current_goal_handle_ = goal_handle;

  // Send goal to raw server, and hook result & feedback
  auto goal = play_motion2_msgs::action::PlayMotion2Raw::Goal();
  goal.motion = loadMotion(goal_handle->get_goal()->motion_name);
  goal.skip_planning = goal_handle->get_goal()->skip_planning;

  auto options =
    rclcpp_action::Client<play_motion2_msgs::action::PlayMotion2Raw>::SendGoalOptions();
  options.result_callback =
    [this, goal_handle](
    rclcpp_action::ClientGoalHandle<play_motion2_msgs::action::PlayMotion2Raw>::WrappedResult
    result) {
      auto result_msg = std::make_shared<play_motion2_msgs::action::PlayMotion2::Result>();
      result_msg->success = result.result->success;
      result_msg->error = result.result->error;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO_STREAM(
            get_logger(), "Motion '" << goal_handle->get_goal()->motion_name <<
              "' completed");
          goal_handle->succeed(result_msg);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO_STREAM(
            get_logger(), "Motion '" << goal_handle->get_goal()->motion_name <<
              "' canceled");
          goal_handle->canceled(result_msg);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR_STREAM(
            get_logger(), "Motion '" << goal_handle->get_goal()->motion_name <<
              "' aborted");
          goal_handle->abort(result_msg);
          break;
        default:
          RCLCPP_ERROR_STREAM(
            get_logger(), "Motion '" << goal_handle->get_goal()->motion_name <<
              "' failed with unknown result code");
          goal_handle->abort(result_msg);
          break;
      }
      current_goal_.reset();
    };
  options.feedback_callback =
    [this, goal_handle](
    rclcpp_action::ClientGoalHandle<play_motion2_msgs::action::PlayMotion2Raw>::SharedPtr,
    play_motion2_msgs::action::PlayMotion2Raw::Feedback::ConstSharedPtr feedback) {
      auto feedback_msg = std::make_shared<play_motion2_msgs::action::PlayMotion2::Feedback>();
      feedback_msg->current_time = feedback->current_time;
      goal_handle->publish_feedback(feedback_msg);
    };
  options.goal_response_callback =
    [this, goal_handle](
    rclcpp_action::ClientGoalHandle<play_motion2_msgs::action::PlayMotion2Raw>::SharedPtr gh)
    {
      if (!gh) {
        RCLCPP_ERROR_STREAM(get_logger(), "Goal was rejected by server");
        auto result_msg = std::make_shared<play_motion2_msgs::action::PlayMotion2::Result>();
        result_msg->success = false;
        result_msg->error = "Goal was rejected by server";
        goal_handle->abort(result_msg);
        return;
      }
      current_goal_ = gh;
      RCLCPP_INFO_STREAM(get_logger(), "Goal accepted by server");
    };

  pm2_raw_action_->async_send_goal(goal, options);
  RCLCPP_INFO_STREAM(get_logger(), "Goal sent to server");
}

bool PlayMotion2Mgr::configureMgr()
{
  motion_loader_ =
    std::make_unique<MotionLoader>(get_logger(), get_node_parameters_interface());
  return motion_loader_->parse_motions();
}

void PlayMotion2Mgr::cleanupMgr()
{
  motion_loader_.reset();
}

const std::vector<std::string> PlayMotion2Mgr::getMotionKeys() const
{
  return motion_loader_->get_motion_keys();
}

bool PlayMotion2Mgr::addMotion(
  const play_motion2_msgs::msg::Motion & motion_msg,
  const bool overwrite)
{
  return motion_loader_->add_motion(motion_msg, overwrite);
}

bool PlayMotion2Mgr::removeMotion(const std::string & motion_key)
{
  return motion_loader_->remove_motion(motion_key);
}

bool PlayMotion2Mgr::motionExists(const std::string & motion_name) const
{
  return motion_loader_->exists(motion_name);
}

play_motion2_msgs::msg::Motion PlayMotion2Mgr::loadMotion(const std::string & motion_name) const
{
  play_motion2_msgs::msg::Motion motion_msg;

  const auto motion_info = motion_loader_->get_motion_info(motion_name);
  motion_msg.key = motion_info.key;
  motion_msg.name = motion_info.name;
  motion_msg.usage = motion_info.usage;
  motion_msg.description = motion_info.description;
  motion_msg.joints = motion_info.joints;
  motion_msg.positions = motion_info.positions;
  motion_msg.times_from_start = motion_info.times;

  return motion_msg;
}
}  // namespace play_motion2
