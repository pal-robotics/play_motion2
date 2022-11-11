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
: Node("play_motion2",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
  motion_keys_({}), motions_({}), list_motions_service_(nullptr)
{
}

void PlayMotion2::init()
{
  parse_controllers(shared_from_this(), controllers_);
  parse_motions(shared_from_this(), motion_keys_, motions_);

  list_motions_service_ = create_service<ListMotions>(
    "play_motion2/list_motions",
    std::bind(
      &PlayMotion2::list_motions_callback,
      this, std::placeholders::_1, std::placeholders::_2));
}

void PlayMotion2::list_motions_callback(
  ListMotions::Request::ConstSharedPtr,
  ListMotions::Response::SharedPtr response)
{
  response->motion_keys = motion_keys_;
}

}  // namespace play_motion2
