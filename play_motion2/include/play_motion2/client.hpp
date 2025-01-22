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
#include <vector>

#include "play_motion2/types.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/msg/motion.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_msgs/srv/remove_motion.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp/node.hpp"

#ifndef PLAY_MOTION2__CLIENT_HPP_
#define PLAY_MOTION2__CLIENT_HPP_

namespace play_motion2
{
class PlayMotion2Client : public rclcpp::Node
{
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
  using MotionMsg = play_motion2_msgs::msg::Motion;

  using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;
  using IsMotionReady = play_motion2_msgs::srv::IsMotionReady;
  using ListMotions = play_motion2_msgs::srv::ListMotions;

  using AddMotion = play_motion2_msgs::srv::AddMotion;
  using RemoveMotion = play_motion2_msgs::srv::RemoveMotion;

public:
  explicit PlayMotion2Client(
    const std::string & name = "play_motion2_client",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlayMotion2Client();

  bool run_motion(
    const std::string & motion_name,
    const bool skip_planning,
    const std::chrono::seconds & motion_timeout = std::chrono::seconds(120));

  bool run_motion_async(
    const std::string & motion_name,
    const bool skip_planning);

  bool is_running_motion() const;
  bool last_succeeded() const;

  std::vector<std::string> list_motions();
  bool is_motion_ready(const std::string & motion_key);
  MotionInfo get_motion_info(const std::string & motion_key);

  bool add_motion(const MotionMsg & motion_msg, const bool overwrite);
  bool remove_motion(const std::string & motion_key);

private:
  const rclcpp_action::ClientGoalHandle<PlayMotion2>::SharedPtr
  send_goal(const std::string & motion_name, const bool skip_planning);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<PlayMotion2>::WrappedResult & result);

private:
  std::atomic_bool running_motion_;
  bool motion_succeeded_;

  rclcpp_action::Client<PlayMotion2>::SharedPtr play_motion2_client_;

  rclcpp::Client<GetMotionInfo>::SharedPtr get_motion_info_client_;
  rclcpp::Client<IsMotionReady>::SharedPtr is_motion_ready_client_;
  rclcpp::Client<ListMotions>::SharedPtr list_motions_client_;

  rclcpp::Client<AddMotion>::SharedPtr add_motion_client_;
  rclcpp::Client<RemoveMotion>::SharedPtr remove_motion_client_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__CLIENT_HPP_
