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

#ifndef PLAY_MOTION2_NODE_TEST_HPP_
#define PLAY_MOTION2_NODE_TEST_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/remove_motion.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"

using std::chrono_literals::operator""s;
const std::chrono::duration TIMEOUT = 10s;

using SwitchController = controller_manager_msgs::srv::SwitchController;

using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;
using ListMotions = play_motion2_msgs::srv::ListMotions;
using IsMotionReady = play_motion2_msgs::srv::IsMotionReady;
using AddMotion = play_motion2_msgs::srv::AddMotion;
using RemoveMotion = play_motion2_msgs::srv::RemoveMotion;

using GoalHandlePM2 = rclcpp_action::ClientGoalHandle<PlayMotion2>::SharedPtr;
using FutureGoalHandlePM2 = std::shared_future<GoalHandlePM2>;

class PlayMotion2NodeTest : public ::testing::Test
{
public:
  PlayMotion2NodeTest() = default;
  ~PlayMotion2NodeTest() = default;

  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp() override;
  void TearDown() override;

  void switch_controllers(
    const std::vector<std::string> & deactivate_list,
    const std::vector<std::string> & activate_list) const;
  void send_pm2_goal(
    const std::string & motion_name, FutureGoalHandlePM2 & future_gh) const;
  void wait_pm2_result(
    const GoalHandlePM2 & future_goal_handle,
    const rclcpp_action::ResultCode & expected_result) const;

  void execute_motion(const std::string & motion_name) const;
  void execute_motion_deactivating_controller_1(
    const std::chrono::seconds & duration,
    const std::string & motion_key,
    const rclcpp_action::ResultCode & code) const;

protected:
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
  rclcpp_action::Client<PlayMotion2>::SharedPtr pm2_action_client_;

private:
  void restore_controllers() const;

  template<typename ClientT>
  void wait_for_controller_service(const ClientT & client) const
  {
    if (!client->wait_for_service(TIMEOUT)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          client_node_->get_logger(),
          "rclcpp interrupted while waiting for the service.");
      } else {
        RCLCPP_ERROR_STREAM(
          client_node_->get_logger(),
          "Service " << client->get_service_name() <<
            " not available. Waiting again...");
      }
      FAIL();
    }
  }
};

#endif  // PLAY_MOTION2_NODE_TEST_HPP_
