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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/transition.hpp"
#include "play_motion2/play_motion2.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_test.hpp"
#include "rclcpp/parameter_client.hpp"

using namespace std::chrono_literals;

namespace play_motion2
{

void PlayMotion2Test::SetUpTestSuite()
{
  rclcpp::init(0, nullptr);
}

void PlayMotion2Test::TearDownTestSuite()
{
  rclcpp::shutdown();
}

void PlayMotion2Test::SetUp()
{
  play_motion2_ = std::make_shared<play_motion2::PlayMotion2>();

  // load parameters
  const auto pkg_path = ament_index_cpp::get_package_share_directory("play_motion2");
  const std::string pm2_config_path = pkg_path + "/test/play_motion2_config.yaml";

  auto synchronous_client =
    std::make_shared<rclcpp::SyncParametersClient>(play_motion2_);
  synchronous_client->load_parameters(pm2_config_path);

  executor_.add_node(play_motion2_->get_node_base_interface());
  runner_ = std::thread([&]() {executor_.spin();});
}

void PlayMotion2Test::TearDown()
{
  executor_.cancel();
  runner_.join();
  play_motion2_.reset();
}

TEST_F(PlayMotion2Test, WrongControllersConfigTest)
{
  // controllers declared empty
  play_motion2_->set_parameter(
    rclcpp::Parameter(
      "controllers",
      std::vector<std::string>{}));

  ASSERT_EQ(
    play_motion2_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).label(), "unconfigured");

  // controllers not declared
  play_motion2_->undeclare_parameter("controllers");

  ASSERT_EQ(
    play_motion2_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).label(), "unconfigured");
}

TEST_F(PlayMotion2Test, WrongMotionsConfigTest)
{
  // void valid motions
  play_motion2_->undeclare_parameter("motions.home.meta.name");
  play_motion2_->undeclare_parameter("motions.pose1.meta.name");

  ASSERT_EQ(
    play_motion2_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).label(), "unconfigured");

  ASSERT_EQ(play_motion2_->get_current_state().label(), "unconfigured");
}

TEST_F(PlayMotion2Test, ListMotionsSrvTest)
{
  ASSERT_EQ(play_motion2_->get_current_state().label(), "unconfigured");

  ASSERT_EQ(
    play_motion2_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).label(), "inactive");

  ASSERT_EQ(
    play_motion2_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).label(), "active");

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto list_motions_client =
    client_node->create_client<play_motion2_msgs::srv::ListMotions>("play_motion2/list_motions");

  ASSERT_TRUE(list_motions_client->wait_for_service(1s));

  auto request = std::make_shared<play_motion2_msgs::srv::ListMotions::Request>();
  auto future_result = list_motions_client->async_send_request(request);

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node, future_result,
      5s), rclcpp::FutureReturnCode::SUCCESS);

  auto result = future_result.get();

  ASSERT_EQ(result->motion_keys.size(), 2);

  std::sort(result->motion_keys.begin(), result->motion_keys.end());
  ASSERT_EQ(result->motion_keys[0], "home");
  ASSERT_EQ(result->motion_keys[1], "pose1");
}

}  // namespace play_motion2
