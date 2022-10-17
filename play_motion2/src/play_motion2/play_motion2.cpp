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
#include "rclcpp/logging.hpp"

namespace play_motion2
{

PlayMotion2::PlayMotion2()
: Node("play_motion2")
{
}

bool PlayMotion2::read_motion_name()
{
  this->declare_parameter<std::string>("motion_name", "");
  this->get_parameter_or<std::string>("motion_name", motion_name_, "");
  RCLCPP_INFO(get_logger(), "MOTION NAME: %s", motion_name_.c_str());
  return 0;
}

}  // namespace play_motion2
