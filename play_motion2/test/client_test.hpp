// Copyright (c) 2026 PAL Robotics S.L. All rights reserved.
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

#ifndef CLIENT_TEST_HPP_
#define CLIENT_TEST_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "play_motion2/client.hpp"
#include "play_motion2/types.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/action/play_motion2_raw.hpp"
#include "play_motion2_msgs/msg/motion.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_msgs/srv/remove_motion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using PlayMotion2Action = play_motion2_msgs::action::PlayMotion2;
using PlayMotion2Raw = play_motion2_msgs::action::PlayMotion2Raw;
using MotionMsg = play_motion2_msgs::msg::Motion;

using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;
using IsMotionReady = play_motion2_msgs::srv::IsMotionReady;
using ListMotions = play_motion2_msgs::srv::ListMotions;
using AddMotion = play_motion2_msgs::srv::AddMotion;
using RemoveMotion = play_motion2_msgs::srv::RemoveMotion;

using GoalHandlePM2 = rclcpp_action::ServerGoalHandle<PlayMotion2Action>;
using GoalHandlePM2Raw = rclcpp_action::ServerGoalHandle<PlayMotion2Raw>;

class ClientTest : public ::testing::Test
{
public:
  ClientTest() = default;
  ~ClientTest() = default;

  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp() override;
  void TearDown() override;

protected:
  // Helper to create a Motion message with the given parameters.
  static MotionMsg make_motion_msg(
    const std::string & key,
    const std::vector<std::string> & joints,
    const std::vector<double> & positions,
    const std::vector<double> & times);

  // Helper to create the executor, add the mock node and client, and spin them concurrently.
  void create_executor_and_spin();

  // Create mock PlayMotion2 action server.
  void create_pm2_action_server();
  // Create mock PlayMotion2Raw action server.
  void create_pm2_raw_action_server();
  // Create mock ListMotions service.
  void create_list_motions_service();
  // Create mock IsMotionReady service.
  void create_is_motion_ready_service();
  // Create mock GetMotionInfo service.
  void create_get_motion_info_service();
  // Create mock AddMotion service.
  void create_add_motion_service();
  // Create mock RemoveMotion service.
  void create_remove_motion_service();

  // Mock server behavior control variables
  bool accept_goal_;
  bool succeed_goal_;

  bool accept_goal_raw_;
  bool succeed_goal_raw_;

  std::vector<std::string> mock_motion_keys_;
  bool mock_is_ready_;
  MotionMsg mock_motion_info_;
  bool mock_add_success_;
  bool mock_remove_success_;

  // Client and node for mocking servers/services
  std::shared_ptr<play_motion2::PlayMotion2Client> client_;
  rclcpp::Node::SharedPtr mock_node_;

  // Executor for spinning the mock node and client concurrently
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

private:
  // Mock servers and services
  rclcpp_action::Server<PlayMotion2Action>::SharedPtr pm2_action_server_;
  rclcpp_action::Server<PlayMotion2Raw>::SharedPtr pm2_raw_action_server_;

  rclcpp::Service<ListMotions>::SharedPtr list_motions_srv_;
  rclcpp::Service<IsMotionReady>::SharedPtr is_motion_ready_srv_;
  rclcpp::Service<GetMotionInfo>::SharedPtr get_motion_info_srv_;
  rclcpp::Service<AddMotion>::SharedPtr add_motion_srv_;
  rclcpp::Service<RemoveMotion>::SharedPtr remove_motion_srv_;

  // Thread for spinning the executor
  std::thread executor_thread_;

  // Mock action server callbacks
  rclcpp_action::GoalResponse handle_goal_pm2(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlayMotion2Action::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel_pm2(
    const std::shared_ptr<GoalHandlePM2> goal_handle);

  void handle_accepted_pm2(const std::shared_ptr<GoalHandlePM2> goal_handle);

  rclcpp_action::GoalResponse handle_goal_pm2_raw(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlayMotion2Raw::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel_pm2_raw(
    const std::shared_ptr<GoalHandlePM2Raw> goal_handle);

  void handle_accepted_pm2_raw(const std::shared_ptr<GoalHandlePM2Raw> goal_handle);
};

#endif  // CLIENT_TEST_HPP_
