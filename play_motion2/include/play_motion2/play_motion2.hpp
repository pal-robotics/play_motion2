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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "play_motion2/play_motion2_helpers.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace play_motion2
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using ListMotions = play_motion2_msgs::srv::ListMotions;

class PlayMotion2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  PlayMotion2();
  ~PlayMotion2() = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  bool init();

  void list_motions_callback(
    ListMotions::Request::ConstSharedPtr request,
    ListMotions::Response::SharedPtr response);

private:
  ControllerList controllers_;
  MotionKeys motion_keys_;
  MotionsMap motions_;

  rclcpp::Service<ListMotions>::SharedPtr list_motions_service_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_HPP_
