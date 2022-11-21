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

namespace play_motion2
{

PlayMotion2::PlayMotion2()
: LifecycleNode("play_motion2",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
  motion_keys_({}),
  motions_({}),
  list_motions_service_(nullptr)
{
}

CallbackReturn PlayMotion2::on_configure(const rclcpp_lifecycle::State & state)
{
  const bool ok =
    parse_controllers(shared_from_this(), controllers_) &&
    parse_motions(shared_from_this(), motion_keys_, motions_);

  if (ok) {
    return CallbackReturn::SUCCESS;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Failed to initialize Play Motion 2");

  return CallbackReturn::FAILURE;
}

CallbackReturn PlayMotion2::on_activate(const rclcpp_lifecycle::State & state)
{
  list_motions_service_ = create_service<ListMotions>(
    "play_motion2/list_motions",
    std::bind(
      &PlayMotion2::list_motions_callback,
      this, std::placeholders::_1, std::placeholders::_2));

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_deactivate(const rclcpp_lifecycle::State & state)
{
  list_motions_service_.reset();
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

}  // namespace play_motion2
