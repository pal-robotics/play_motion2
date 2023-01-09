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

#include <chrono>

#include "gtest/gtest.h"

#include "play_motion2_node_test.hpp"
#include "rclcpp/create_client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/create_client.hpp"

namespace play_motion2
{

void PlayMotion2NodeTest::SetUpTestSuite()
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("set_up_node");
  auto client = node->create_client<IsMotionReady>(
    "play_motion2/is_motion_ready");

  auto request = std::make_shared<IsMotionReady::Request>();
  request->motion_key = "home";

  using IsMotionReadyFutureAndRequestId = rclcpp::Client<IsMotionReady>::FutureAndRequestId;
  std::shared_ptr<IsMotionReadyFutureAndRequestId> future_result;
  unsigned int retries = 3u;
  do {
    future_result =
      std::make_shared<IsMotionReadyFutureAndRequestId>(client->async_send_request(request));
    retries--;
  } while (retries >= 0 && rclcpp::spin_until_future_complete(
    node, *future_result,
    TIMEOUT) != rclcpp::FutureReturnCode::SUCCESS);

  ASSERT_NE(retries, 0) << "Timeout while waiting for motions to be ready";
}

void PlayMotion2NodeTest::TearDownTestSuite()
{
  rclcpp::shutdown();
}

void PlayMotion2NodeTest::SetUp()
{
  client_node_ = rclcpp::Node::make_shared("pm2_client_node");

  pm2_action_client_ = rclcpp_action::create_client<PlayMotion2>(
    client_node_, "play_motion2");

  switch_controller_client_ = client_node_->create_client<SwitchController>(
    "controller_manager/switch_controller");

  ASSERT_NO_THROW(restore_controllers());
}

void PlayMotion2NodeTest::TearDown()
{
  switch_controller_client_.reset();
  pm2_action_client_.reset();
  client_node_.reset();
}

void PlayMotion2NodeTest::restore_controllers() const
{
  wait_for_controller_service(switch_controller_client_);

  // reactivate controllers
  const std::vector<std::string> controllers_list =
  {"joint_state_broadcaster", "controller_1", "controller_2"};

  for (const auto & controller : controllers_list) {
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = {controller};
    request->strictness = SwitchController::Request::BEST_EFFORT;

    auto result = switch_controller_client_->async_send_request(request);

    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        client_node_, result,
        TIMEOUT), rclcpp::FutureReturnCode::SUCCESS);

    ASSERT_TRUE(result.get()->ok);
  }
}

void PlayMotion2NodeTest::deactivate_controllers(const std::vector<std::string> controllers_list)
const
{
  auto deactivate_request = std::make_shared<SwitchController::Request>();
  deactivate_request->deactivate_controllers = controllers_list;
  deactivate_request->strictness = SwitchController::Request::BEST_EFFORT;
  auto deactivate_future_result = switch_controller_client_->async_send_request(deactivate_request);

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node_, deactivate_future_result,
      TIMEOUT), rclcpp::FutureReturnCode::SUCCESS);

  auto deactivate_result = deactivate_future_result.get();
  ASSERT_TRUE(deactivate_result->ok);
}

void PlayMotion2NodeTest::send_pm2_goal(
  const std::string & motion_name,
  FutureGoalHandlePM2 & future_gh) const
{
  auto pm2_goal = PlayMotion2::Goal();
  pm2_goal.motion_name = motion_name;
  future_gh = pm2_action_client_->async_send_goal(pm2_goal);

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node_, future_gh,
      TIMEOUT), rclcpp::FutureReturnCode::SUCCESS);
}

void PlayMotion2NodeTest::wait_pm2_result(
  GoalHandlePM2 future_goal_handle,
  rclcpp_action::ResultCode expected_result) const
{
  auto result_future = pm2_action_client_->async_get_result(future_goal_handle);

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node_, result_future), rclcpp::FutureReturnCode::SUCCESS);

  auto result = result_future.get();

  ASSERT_EQ(result.code, expected_result);
}

TEST_F(PlayMotion2NodeTest, ListMotionsSrvTest)
{
  auto list_motions_client =
    client_node_->create_client<ListMotions>("play_motion2/list_motions");

  ASSERT_TRUE(list_motions_client->wait_for_service(TIMEOUT));

  auto request = std::make_shared<ListMotions::Request>();
  auto future_result = list_motions_client->async_send_request(request);

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node_, future_result,
      TIMEOUT), rclcpp::FutureReturnCode::SUCCESS);

  const auto result = future_result.get();

  ASSERT_EQ(result->motion_keys.size(), 2);

  std::sort(result->motion_keys.begin(), result->motion_keys.end());
  ASSERT_EQ(result->motion_keys[0], "home");
  ASSERT_EQ(result->motion_keys[1], "pose1");
}

TEST_F(PlayMotion2NodeTest, BadMotionName)
{
  FutureGoalHandlePM2 goal_handle_future;
  send_pm2_goal("unreal_motion", goal_handle_future);

  const auto goal_handle = goal_handle_future.get();

  // no goal_handle means goal has been rejected
  ASSERT_FALSE(goal_handle);
}

TEST_F(PlayMotion2NodeTest, MalformedMotion)
{
  FutureGoalHandlePM2 goal_handle_future;
  send_pm2_goal("malformed_motion", goal_handle_future);

  auto goal_handle = goal_handle_future.get();

  // no goal_handle means goal has been rejected
  ASSERT_FALSE(goal_handle);
}

TEST_F(PlayMotion2NodeTest, ControllerDeactivated)
{
  // deactivate controller_1
  deactivate_controllers({"controller_1"});

  // create and send goal
  FutureGoalHandlePM2 goal_handle_future;
  send_pm2_goal("home", goal_handle_future);

  const auto goal_handle = goal_handle_future.get();

  // no goal_handle means goal has been rejected
  ASSERT_FALSE(goal_handle);
}

TEST_F(PlayMotion2NodeTest, ExecuteSuccessfulMotion)
{
  // create and send goal
  FutureGoalHandlePM2 goal_handle_future;
  send_pm2_goal("home", goal_handle_future);

  auto goal_handle = goal_handle_future.get();

  ASSERT_TRUE(goal_handle);

  // wait for result
  wait_pm2_result(goal_handle, rclcpp_action::ResultCode::SUCCEEDED);
}

void PlayMotion2NodeTest::execute_failing_motion(std::chrono::seconds duration) const
{
  // create and send goal
  FutureGoalHandlePM2 goal_handle_future;
  send_pm2_goal("pose1", goal_handle_future);

  auto goal_handle = goal_handle_future.get();

  ASSERT_TRUE(goal_handle);

  // sleep to give time to send all follow_joint_trajectory goals
  std::this_thread::sleep_for(duration);

  // deactivate controller_1 to make motion fail
  deactivate_controllers({"controller_1"});

  // wait for result
  wait_pm2_result(goal_handle, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(PlayMotion2NodeTest, ControllersChangedBeforeExecution)
{
  execute_failing_motion(0s);
}

TEST_F(PlayMotion2NodeTest, ControllersChangedDuringExecution)
{
  execute_failing_motion(1s);
}

}  // namespace play_motion2
