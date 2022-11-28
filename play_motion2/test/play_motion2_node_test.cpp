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

#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_node_test.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"

using namespace std::chrono_literals;

namespace play_motion2
{

void PlayMotion2NodeTest::SetUpTestSuite()
{
  rclcpp::init(0, nullptr);
}

void PlayMotion2NodeTest::TearDownTestSuite()
{
  rclcpp::shutdown();
}

void PlayMotion2NodeTest::SetUp()
{
  client_node_ = rclcpp::Node::make_shared("pm2_client_node");
}

void PlayMotion2NodeTest::TearDown()
{
  client_node_.reset();
}

TEST_F(PlayMotion2NodeTest, ListMotionsSrvTest)
{
  auto list_motions_client =
    client_node_->create_client<play_motion2_msgs::srv::ListMotions>("play_motion2/list_motions");

  ASSERT_TRUE(list_motions_client->wait_for_service(1s));

  auto request = std::make_shared<play_motion2_msgs::srv::ListMotions::Request>();
  auto future_result = list_motions_client->async_send_request(request);

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node_, future_result,
      5s), rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();

  ASSERT_EQ(result->motion_keys.size(), 2);

  std::sort(result->motion_keys.begin(), result->motion_keys.end());
  ASSERT_EQ(result->motion_keys[0], "home");
  ASSERT_EQ(result->motion_keys[1], "pose1");
}

}  // namespace play_motion2
