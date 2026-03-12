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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "client_test.hpp"

#include "play_motion2/client.hpp"
#include "play_motion2/types.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

void ClientTest::SetUpTestSuite()
{
  rclcpp::init(0, nullptr);
}

void ClientTest::TearDownTestSuite()
{
  rclcpp::shutdown();
}

void ClientTest::SetUp()
{
  // --- reset configurable mock-behaviour flags ---
  accept_goal_ = true;
  succeed_goal_ = true;
  accept_goal_raw_ = true;
  succeed_goal_raw_ = true;
  mock_motion_keys_ = {"home", "pose1"};
  mock_is_ready_ = true;
  mock_add_success_ = true;
  mock_remove_success_ = true;

  // Populate a default mock_motion_info_
  mock_motion_info_.key = "home";
  mock_motion_info_.joints = {"joint1", "joint2"};
  mock_motion_info_.positions = {0.5, 0.5};
  mock_motion_info_.times_from_start = {0.1};
  mock_motion_info_.name = "home";
  mock_motion_info_.usage = "test";
  mock_motion_info_.description = "default home pose";

  // --- create mock node and client (servers/services are created on demand) ---
  mock_node_ = rclcpp::Node::make_shared("mock_pm2_server");
  client_ = std::make_shared<play_motion2::PlayMotion2Client>("test_pm2_client");
}

void ClientTest::TearDown()
{
  if (executor_) {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  pm2_action_server_.reset();
  pm2_raw_action_server_.reset();
  list_motions_srv_.reset();
  is_motion_ready_srv_.reset();
  get_motion_info_srv_.reset();
  add_motion_srv_.reset();
  remove_motion_srv_.reset();

  client_.reset();
  mock_node_.reset();
  executor_.reset();
}

void ClientTest::create_executor_and_spin()
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(mock_node_);
  executor_thread_ = std::thread([this]() {executor_->spin();});

  const auto start = std::chrono::steady_clock::now();
  while (!executor_->is_spinning() &&
    (std::chrono::steady_clock::now() - start) < 5s)
  {
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(executor_->is_spinning()) << "Executor failed to start spinning within timeout";
}

void ClientTest::create_pm2_action_server()
{
  pm2_action_server_ = rclcpp_action::create_server<PlayMotion2Action>(
    mock_node_, "/play_motion2",
    std::bind(&ClientTest::handle_goal_pm2, this, _1, _2),
    std::bind(&ClientTest::handle_cancel_pm2, this, _1),
    std::bind(&ClientTest::handle_accepted_pm2, this, _1));
}

void ClientTest::create_pm2_raw_action_server()
{
  pm2_raw_action_server_ = rclcpp_action::create_server<PlayMotion2Raw>(
    mock_node_, "/play_motion2/raw",
    std::bind(&ClientTest::handle_goal_pm2_raw, this, _1, _2),
    std::bind(&ClientTest::handle_cancel_pm2_raw, this, _1),
    std::bind(&ClientTest::handle_accepted_pm2_raw, this, _1));
}

void ClientTest::create_list_motions_service()
{
  list_motions_srv_ = mock_node_->create_service<ListMotions>(
    "/play_motion2/list_motions",
    [this](
      const ListMotions::Request::SharedPtr /*request*/,
      ListMotions::Response::SharedPtr response)
    {
      response->motion_keys = mock_motion_keys_;
    });
}

void ClientTest::create_is_motion_ready_service()
{
  is_motion_ready_srv_ = mock_node_->create_service<IsMotionReady>(
    "/play_motion2/is_motion_ready",
    [this](
      const IsMotionReady::Request::SharedPtr /*request*/,
      IsMotionReady::Response::SharedPtr response)
    {
      response->is_ready = mock_is_ready_;
    });
}

void ClientTest::create_get_motion_info_service()
{
  get_motion_info_srv_ = mock_node_->create_service<GetMotionInfo>(
    "/play_motion2/get_motion_info",
    [this](
      const GetMotionInfo::Request::SharedPtr /*request*/,
      GetMotionInfo::Response::SharedPtr response)
    {
      response->motion = mock_motion_info_;
    });
}

void ClientTest::create_add_motion_service()
{
  add_motion_srv_ = mock_node_->create_service<AddMotion>(
    "/play_motion2/add_motion",
    [this](
      const AddMotion::Request::SharedPtr /*request*/,
      AddMotion::Response::SharedPtr response)
    {
      response->success = mock_add_success_;
    });
}

void ClientTest::create_remove_motion_service()
{
  remove_motion_srv_ = mock_node_->create_service<RemoveMotion>(
    "/play_motion2/remove_motion",
    [this](
      const RemoveMotion::Request::SharedPtr /*request*/,
      RemoveMotion::Response::SharedPtr response)
    {
      response->success = mock_remove_success_;
    });
}

//  Helpers
MotionMsg ClientTest::make_motion_msg(
  const std::string & key,
  const std::vector<std::string> & joints,
  const std::vector<double> & positions,
  const std::vector<double> & times)
{
  MotionMsg msg;
  msg.key = key;
  msg.joints = joints;
  msg.positions = positions;
  msg.times_from_start = times;
  return msg;
}

//  Action-server callbacks – PlayMotion2
rclcpp_action::GoalResponse ClientTest::handle_goal_pm2(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlayMotion2Action::Goal>/*goal*/)
{
  return accept_goal_ ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ClientTest::handle_cancel_pm2(
  const std::shared_ptr<GoalHandlePM2>/*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ClientTest::handle_accepted_pm2(const std::shared_ptr<GoalHandlePM2> goal_handle)
{
  auto result = std::make_shared<PlayMotion2Action::Result>();
  if (succeed_goal_) {
    result->success = true;
    goal_handle->succeed(result);
  } else {
    result->success = false;
    result->error = "mock failure";
    goal_handle->abort(result);
  }
}

rclcpp_action::GoalResponse ClientTest::handle_goal_pm2_raw(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlayMotion2Raw::Goal>/*goal*/)
{
  return accept_goal_raw_ ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ClientTest::handle_cancel_pm2_raw(
  const std::shared_ptr<GoalHandlePM2Raw>/*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ClientTest::handle_accepted_pm2_raw(const std::shared_ptr<GoalHandlePM2Raw> goal_handle)
{
  std::thread(
    [this, goal_handle]()
    {
      auto result = std::make_shared<PlayMotion2Raw::Result>();
      if (succeed_goal_raw_) {
        result->success = true;
        goal_handle->succeed(result);
      } else {
        result->success = false;
        result->error = "mock raw failure";
        goal_handle->abort(result);
      }
    }).detach();
}

//  =============================
//  TESTS
//  =============================

TEST_F(ClientTest, DefaultConstruction)
{
  // Verify that the client was created with the expected default node name.
  auto default_client = std::make_shared<play_motion2::PlayMotion2Client>();
  EXPECT_EQ(std::string(default_client->get_name()), "play_motion2_client");
}

TEST_F(ClientTest, CustomNameConstruction)
{
  EXPECT_EQ(std::string(client_->get_name()), "test_pm2_client");
}

TEST_F(ClientTest, ListMotionsReturnsExpectedKeys)
{
  create_list_motions_service();
  create_executor_and_spin();

  const auto motions = client_->list_motions();
  ASSERT_EQ(motions.size(), 2u);
  EXPECT_EQ(motions[0], "home");
  EXPECT_EQ(motions[1], "pose1");
}

TEST_F(ClientTest, ListMotionsReturnsEmptyWhenNoMotions)
{
  create_list_motions_service();
  create_executor_and_spin();

  mock_motion_keys_.clear();
  const auto motions = client_->list_motions();
  EXPECT_TRUE(motions.empty());
}

TEST_F(ClientTest, IsMotionReadyReturnsTrue)
{
  create_is_motion_ready_service();
  create_executor_and_spin();

  mock_is_ready_ = true;
  EXPECT_TRUE(client_->is_motion_ready("home"));
}

TEST_F(ClientTest, IsMotionReadyReturnsFalse)
{
  create_is_motion_ready_service();
  create_executor_and_spin();

  mock_is_ready_ = false;
  EXPECT_FALSE(client_->is_motion_ready("home"));
}

TEST_F(ClientTest, GetMotionInfoReturnsCorrectData)
{
  create_get_motion_info_service();
  create_executor_and_spin();

  const auto info = client_->get_motion_info("home");

  EXPECT_EQ(info.key, "home");

  ASSERT_EQ(info.joints.size(), 2u);
  EXPECT_EQ(info.joints[0], "joint1");
  EXPECT_EQ(info.joints[1], "joint2");

  ASSERT_EQ(info.positions.size(), 2u);
  EXPECT_DOUBLE_EQ(info.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(info.positions[1], 0.5);

  ASSERT_EQ(info.times.size(), 1u);
  EXPECT_DOUBLE_EQ(info.times[0], 0.1);
  EXPECT_EQ(info.name, "home");
  EXPECT_EQ(info.usage, "test");
  EXPECT_EQ(info.description, "default home pose");
}

TEST_F(ClientTest, GetMotionInfoWithMeta)
{
  create_get_motion_info_service();
  create_executor_and_spin();

  mock_motion_info_.name = "wave";
  mock_motion_info_.usage = "greeting";
  mock_motion_info_.description = "wave hello";

  const auto info = client_->get_motion_info("wave");
  EXPECT_EQ(info.name, "wave");
  EXPECT_EQ(info.usage, "greeting");
  EXPECT_EQ(info.description, "wave hello");
}

TEST_F(ClientTest, GetMotionInfoNoMeta)
{
  create_get_motion_info_service();
  create_executor_and_spin();

  mock_motion_info_.name = "";
  mock_motion_info_.usage = "";
  mock_motion_info_.description = "";

  const auto info = client_->get_motion_info("pose1");
  EXPECT_TRUE(info.name.empty());
  EXPECT_TRUE(info.usage.empty());
  EXPECT_TRUE(info.description.empty());
}

TEST_F(ClientTest, AddMotionSuccess)
{
  create_add_motion_service();
  create_executor_and_spin();

  mock_add_success_ = true;
  const auto motion = make_motion_msg("new_pose", {"j1"}, {1.0}, {0.5});
  EXPECT_TRUE(client_->add_motion(motion, false));
}

TEST_F(ClientTest, AddMotionFailure)
{
  create_add_motion_service();
  create_executor_and_spin();

  mock_add_success_ = false;
  const auto motion = make_motion_msg("bad_pose", {"j1"}, {1.0}, {0.5});
  EXPECT_FALSE(client_->add_motion(motion, false));
}

TEST_F(ClientTest, AddMotionWithOverwrite)
{
  create_add_motion_service();
  create_executor_and_spin();

  mock_add_success_ = true;
  const auto motion = make_motion_msg("home", {"j1", "j2"}, {0.1, 0.2}, {1.0});
  EXPECT_TRUE(client_->add_motion(motion, true));
}

TEST_F(ClientTest, RemoveMotionSuccess)
{
  create_remove_motion_service();
  create_executor_and_spin();

  mock_remove_success_ = true;
  EXPECT_TRUE(client_->remove_motion("home"));
}

TEST_F(ClientTest, RemoveMotionFailure)
{
  create_remove_motion_service();
  create_executor_and_spin();

  mock_remove_success_ = false;
  EXPECT_FALSE(client_->remove_motion("nonexistent"));
}

TEST_F(ClientTest, RunMotionSyncSuccess)
{
  create_pm2_action_server();
  create_executor_and_spin();

  succeed_goal_ = true;
  EXPECT_TRUE(client_->run_motion("home", true));
  // After completing, is_running_motion should be false
  EXPECT_FALSE(client_->is_running_motion());
  EXPECT_TRUE(client_->last_succeeded());
}

TEST_F(ClientTest, RunMotionSyncAborted)
{
  create_pm2_action_server();
  create_executor_and_spin();

  succeed_goal_ = false;
  EXPECT_FALSE(client_->run_motion("home", true));
  EXPECT_FALSE(client_->is_running_motion());
  EXPECT_FALSE(client_->last_succeeded());
}

TEST_F(ClientTest, RunMotionSyncRejected)
{
  create_pm2_action_server();
  create_executor_and_spin();

  accept_goal_ = false;
  EXPECT_FALSE(client_->run_motion("rejected_motion", true));
}

TEST_F(ClientTest, RunMotionSyncSkipPlanningFalse)
{
  create_pm2_action_server();
  create_executor_and_spin();

  succeed_goal_ = true;
  EXPECT_TRUE(client_->run_motion("home", false));
  EXPECT_TRUE(client_->last_succeeded());
}

TEST_F(ClientTest, RunMotionAsyncSuccess)
{
  create_pm2_action_server();
  create_executor_and_spin();

  succeed_goal_ = true;
  EXPECT_TRUE(client_->run_motion_async("home", true));

  // Add client to executor after send_goal completes
  executor_->add_node(client_);

  // The motion should be marked as running
  EXPECT_TRUE(client_->is_running_motion());

  // Wait for completion
  const auto start = std::chrono::steady_clock::now();
  while (client_->is_running_motion() &&
    (std::chrono::steady_clock::now() - start) < 5s)
  {
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_FALSE(client_->is_running_motion());
  EXPECT_TRUE(client_->last_succeeded());
}

TEST_F(ClientTest, RunMotionAsyncRejected)
{
  create_pm2_action_server();
  create_executor_and_spin();

  accept_goal_ = false;
  EXPECT_FALSE(client_->run_motion_async("bad", true));
}

TEST_F(ClientTest, RunMotionAsyncAborted)
{
  create_pm2_action_server();
  create_executor_and_spin();

  succeed_goal_ = false;
  EXPECT_TRUE(client_->run_motion_async("home", true));

  // Add client to executor after send_goal completes
  executor_->add_node(client_);

  const auto start = std::chrono::steady_clock::now();
  while (client_->is_running_motion() &&
    (std::chrono::steady_clock::now() - start) < 5s)
  {
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_FALSE(client_->is_running_motion());
  EXPECT_FALSE(client_->last_succeeded());
}

TEST_F(ClientTest, RunRawMotionSyncSuccess)
{
  create_pm2_raw_action_server();
  create_executor_and_spin();

  succeed_goal_raw_ = true;
  const auto motion = make_motion_msg("raw_test", {"j1"}, {1.0}, {0.5});
  EXPECT_TRUE(client_->run_motion(motion, true));
  EXPECT_FALSE(client_->is_running_motion());
  EXPECT_TRUE(client_->last_succeeded());
}

TEST_F(ClientTest, RunRawMotionSyncAborted)
{
  create_pm2_raw_action_server();
  create_executor_and_spin();

  succeed_goal_raw_ = false;
  const auto motion = make_motion_msg("raw_fail", {"j1"}, {1.0}, {0.5});
  EXPECT_FALSE(client_->run_motion(motion, true));
  EXPECT_FALSE(client_->is_running_motion());
  EXPECT_FALSE(client_->last_succeeded());
}

TEST_F(ClientTest, RunRawMotionSyncRejected)
{
  create_pm2_raw_action_server();
  create_executor_and_spin();

  accept_goal_raw_ = false;
  const auto motion = make_motion_msg("raw_rej", {"j1"}, {1.0}, {0.5});
  EXPECT_FALSE(client_->run_motion(motion, true));
}

TEST_F(ClientTest, RunRawMotionAsyncSuccess)
{
  create_pm2_raw_action_server();
  create_executor_and_spin();

  succeed_goal_raw_ = true;
  const auto motion = make_motion_msg("async_raw", {"j1", "j2"}, {0.1, 0.2}, {1.0});
  EXPECT_TRUE(client_->run_motion_async(motion, true));

  // Add client to executor after send_goal_raw completes
  executor_->add_node(client_);

  EXPECT_TRUE(client_->is_running_motion());

  const auto start = std::chrono::steady_clock::now();
  while (client_->is_running_motion() &&
    (std::chrono::steady_clock::now() - start) < 5s)
  {
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_FALSE(client_->is_running_motion());
  EXPECT_TRUE(client_->last_succeeded());
}

TEST_F(ClientTest, RunRawMotionAsyncRejected)
{
  create_pm2_raw_action_server();
  create_executor_and_spin();

  accept_goal_raw_ = false;
  const auto motion = make_motion_msg("async_raw_rej", {"j1"}, {0.1}, {1.0});
  EXPECT_FALSE(client_->run_motion_async(motion, true));
}

TEST_F(ClientTest, InitialStateNotRunning)
{
  EXPECT_FALSE(client_->is_running_motion());
}

TEST_F(ClientTest, LastSucceededFalseInitially)
{
  // No motion has been run, so last_succeeded should be false.
  EXPECT_FALSE(client_->last_succeeded());
}

TEST_F(ClientTest, SequentialSyncMotions)
{
  create_pm2_action_server();
  create_executor_and_spin();

  succeed_goal_ = true;
  EXPECT_TRUE(client_->run_motion("home", true));
  EXPECT_TRUE(client_->last_succeeded());

  EXPECT_TRUE(client_->run_motion("pose1", true));
  EXPECT_TRUE(client_->last_succeeded());

  succeed_goal_ = false;
  EXPECT_FALSE(client_->run_motion("home", true));
  EXPECT_FALSE(client_->last_succeeded());

  succeed_goal_ = true;
  EXPECT_TRUE(client_->run_motion("pose1", true));
  EXPECT_TRUE(client_->last_succeeded());
}

TEST_F(ClientTest, RunRawMotionMultipleJoints)
{
  create_pm2_raw_action_server();
  create_executor_and_spin();

  succeed_goal_raw_ = true;
  const auto motion = make_motion_msg(
    "multi_joint", {"j1", "j2", "j3", "j4"},
    {0.0, 1.0, 2.0, 3.0, 0.5, 1.5, 2.5, 3.5},
    {1.0, 2.0});
  EXPECT_TRUE(client_->run_motion(motion, true));
}

TEST_F(ClientTest, RunRawMotionSinglePoint)
{
  create_pm2_raw_action_server();
  create_executor_and_spin();

  succeed_goal_raw_ = true;
  const auto motion = make_motion_msg("single_pt", {"j1"}, {0.0}, {1.0});
  EXPECT_TRUE(client_->run_motion(motion, true));
}
