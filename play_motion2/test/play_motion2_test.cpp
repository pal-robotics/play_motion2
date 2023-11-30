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
#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/transition.hpp"
#include "play_motion2/play_motion2.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_test.hpp"
#include "rclcpp/parameter_client.hpp"


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
  const auto test_path = std::filesystem::absolute(__FILE__).parent_path().string();
  const std::string pm2_config_path = test_path + "/play_motion2_config.yaml";

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

TEST_F(PlayMotion2Test, WrongMotionsConfigTest)
{
  // void valid motions
  play_motion2_->undeclare_parameter("motions.home.joints");
  play_motion2_->undeclare_parameter("motions.pose1.times_from_start");

  ASSERT_EQ(
    play_motion2_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).label(), "unconfigured");
}
