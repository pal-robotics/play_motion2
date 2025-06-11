// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

#include <string>

#include "play_motion2/client.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"

#include "rclcpp_action/create_client.hpp"
#include "rclcpp/executors.hpp"


namespace play_motion2
{
using namespace std::chrono_literals;
using std::placeholders::_1;
const auto kTimeout = 10s;

PlayMotion2Client::PlayMotion2Client(const std::string & name, const rclcpp::NodeOptions & options)
: Node(name, options)
  , running_motion_(false)
  , motion_succeeded_(false)
{
  play_motion2_client_ = rclcpp_action::create_client<PlayMotion2>(this, "/play_motion2");
  play_motion2_client_raw_ =
    rclcpp_action::create_client<PlayMotion2Raw>(this, "/play_motion2/raw");

  get_motion_info_client_ = this->create_client<GetMotionInfo>("/play_motion2/get_motion_info");
  is_motion_ready_client_ = this->create_client<IsMotionReady>("/play_motion2/is_motion_ready");
  list_motions_client_ = this->create_client<ListMotions>("/play_motion2/list_motions");

  add_motion_client_ = this->create_client<AddMotion>("/play_motion2/add_motion");
  remove_motion_client_ = this->create_client<RemoveMotion>("/play_motion2/remove_motion");
}

PlayMotion2Client::~PlayMotion2Client()
{
  play_motion2_client_.reset();
  play_motion2_client_raw_.reset();

  get_motion_info_client_.reset();
  is_motion_ready_client_.reset();
  list_motions_client_.reset();
  add_motion_client_.reset();
  remove_motion_client_.reset();
}

const rclcpp_action::ClientGoalHandle<play_motion2_msgs::action::PlayMotion2>::SharedPtr
PlayMotion2Client::send_goal(const std::string & motion_name, const bool skip_planning)
{
  if (!play_motion2_client_->wait_for_action_server(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for PlayMotion2 server");
    return nullptr;
  }

  auto goal = PlayMotion2::Goal();
  goal.motion_name = motion_name;
  goal.skip_planning = skip_planning;

  auto send_goal_options = rclcpp_action::Client<PlayMotion2>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&PlayMotion2Client::result_callback, this, _1);

  auto goal_future = play_motion2_client_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot send goal to PlayMotion2 server");
    return nullptr;
  }

  const auto goal_handle = goal_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by PlayMotion2 server");
    return nullptr;
  }

  return goal_handle;
}

void PlayMotion2Client::result_callback(
  const rclcpp_action::ClientGoalHandle<PlayMotion2>::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    motion_succeeded_ = true;
    RCLCPP_INFO(this->get_logger(), "Motion execution completed");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Motion execution failed");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(this->get_logger(), "Motion execution cancelled");
  } else {
    RCLCPP_FATAL(this->get_logger(), "Got unknown result code");
  }
  running_motion_ = false;
}

const rclcpp_action::ClientGoalHandle<play_motion2_msgs::action::PlayMotion2Raw>::SharedPtr
PlayMotion2Client::send_goal_raw(
  const play_motion2_msgs::msg::Motion & motion,
  const bool skip_planning)
{
  if (!play_motion2_client_raw_->wait_for_action_server(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for PlayMotion2 server");
    return nullptr;
  }

  auto goal = PlayMotion2Raw::Goal();
  goal.motion = motion;
  goal.skip_planning = skip_planning;

  auto send_goal_options = rclcpp_action::Client<PlayMotion2Raw>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&PlayMotion2Client::result_callback_raw, this, _1);

  auto goal_future = play_motion2_client_raw_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot send goal to PlayMotion2 server");
    return nullptr;
  }

  const auto goal_handle = goal_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by PlayMotion2 server");
    return nullptr;
  }

  return goal_handle;
}

void PlayMotion2Client::result_callback_raw(
  const rclcpp_action::ClientGoalHandle<PlayMotion2Raw>::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    motion_succeeded_ = true;
    RCLCPP_INFO(this->get_logger(), "Motion execution completed");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(this->get_logger(), "Motion execution failed");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(this->get_logger(), "Motion execution cancelled");
  } else {
    RCLCPP_FATAL(this->get_logger(), "Got unknown result code");
  }
  running_motion_ = false;
}

bool PlayMotion2Client::run_motion(
  const std::string & motion_name,
  const bool skip_planning,
  const std::chrono::seconds & motion_timeout)
{
  auto goal_handle = send_goal(motion_name, skip_planning);
  if (!goal_handle) {
    return false;
  }
  motion_succeeded_ = false;
  running_motion_ = true;

  auto execution_future = play_motion2_client_->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(
      shared_from_this(), execution_future, motion_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute PlayMotion2 goal");
    play_motion2_client_->async_cancel_goal(goal_handle);
    running_motion_ = false;
    return false;
  }

  // Wait for the result callback to set the motion_succeeded_ flag
  while (running_motion_) {
    std::this_thread::sleep_for(100ms);
  }

  return motion_succeeded_;
}

bool PlayMotion2Client::run_motion_async(
  const std::string & motion_name,
  const bool skip_planning)
{
  const auto goal_handle = send_goal(motion_name, skip_planning);
  if (!goal_handle) {
    return false;
  }

  motion_succeeded_ = false;
  running_motion_ = true;
  return true;
}

bool PlayMotion2Client::run_motion(
  const play_motion2_msgs::msg::Motion & motion,
  const bool skip_planning,
  const std::chrono::seconds & motion_timeout)
{
  auto goal_handle = send_goal_raw(motion, skip_planning);
  if (!goal_handle) {
    return false;
  }
  motion_succeeded_ = false;
  running_motion_ = true;

  auto execution_future = play_motion2_client_raw_->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(
      shared_from_this(), execution_future, motion_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute PlayMotion2 goal");
    play_motion2_client_raw_->async_cancel_goal(goal_handle);
    running_motion_ = false;
    return false;
  }

  // Wait for the result callback to set the motion_succeeded_ flag
  while (running_motion_) {
    std::this_thread::sleep_for(100ms);
  }

  return motion_succeeded_;
}

bool PlayMotion2Client::run_motion_async(
  const play_motion2_msgs::msg::Motion & motion,
  const bool skip_planning)
{
  const auto goal_handle = send_goal_raw(motion, skip_planning);
  if (!goal_handle) {
    return false;
  }

  motion_succeeded_ = false;
  running_motion_ = true;
  return true;
}

bool PlayMotion2Client::is_running_motion() const
{
  return running_motion_;
}

bool PlayMotion2Client::last_succeeded() const
{
  if (running_motion_) {
    RCLCPP_WARN(this->get_logger(), "Motion is still running.");
  }

  return motion_succeeded_;
}

std::vector<std::string> PlayMotion2Client::list_motions()
{
  std::vector<std::string> motions;

  if (!list_motions_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for ListMotions service");
    return motions;
  }

  auto request = std::make_shared<ListMotions::Request>();
  auto future = list_motions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to list motions");
    return motions;
  }

  return future.get()->motion_keys;
}

bool PlayMotion2Client::is_motion_ready(const std::string & motion_key)
{
  if (!is_motion_ready_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for IsMotionReady service");
    return false;
  }

  auto request = std::make_shared<IsMotionReady::Request>();
  request->motion_key = motion_key;
  auto future = is_motion_ready_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to check if motion is ready");
    return false;
  }

  return future.get()->is_ready;
}

MotionInfo PlayMotion2Client::get_motion_info(const std::string & motion_key)
{
  if (!get_motion_info_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for GetMotionInfo service");
    return MotionInfo();
  }

  auto request = std::make_shared<GetMotionInfo::Request>();
  request->motion_key = motion_key;
  auto future = get_motion_info_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get motion info");
    return MotionInfo();
  }

  auto motion = future.get()->motion;

  MotionInfo motion_info;
  motion_info.key = motion.key;
  motion_info.joints = motion.joints;
  motion_info.positions = motion.positions;
  motion_info.times = motion.times_from_start;
  motion_info.name = motion.name;
  motion_info.description = motion.description;
  motion_info.usage = motion.usage;

  return motion_info;
}

bool PlayMotion2Client::add_motion(const MotionMsg & motion_msg, const bool overwrite)
{
  if (!add_motion_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for AddMotion service");
    return false;
  }

  auto request = std::make_shared<AddMotion::Request>();
  request->motion = motion_msg;
  request->overwrite = overwrite;
  auto future = add_motion_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to add motion");
    return false;
  }

  return future.get()->success;
}

bool PlayMotion2Client::remove_motion(const std::string & motion_key)
{
  if (!remove_motion_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for RemoveMotion service");
    return false;
  }

  auto request = std::make_shared<RemoveMotion::Request>();
  request->motion_key = motion_key;
  auto future = remove_motion_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to remove motion");
    return false;
  }

  return future.get()->success;
}
}  // namespace play_motion2
