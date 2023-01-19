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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"

#include "play_motion2_helpers_test.hpp"
#include "play_motion2/play_motion2_helpers.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace play_motion2
{

void PlayMotion2HelpersTest::SetUpTestSuite()
{
  rclcpp::init(0, nullptr);
}

void PlayMotion2HelpersTest::TearDownTestSuite()
{
  rclcpp::shutdown();
}

void PlayMotion2HelpersTest::SetUp()
{
  node_ = std::make_shared<rclcpp::Node>(
    "play_motion2_helpers_test",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true));

  lifecycle_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "play_motion2_lifecycle_helpers_test",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true));

  // load parameters
  const auto pkg_path = ament_index_cpp::get_package_share_directory("play_motion2");
  const std::string config_path = pkg_path + "/test/config.yaml";

  auto synchronous_client =
    std::make_shared<rclcpp::SyncParametersClient>(node_);
  synchronous_client->load_parameters(config_path);

  auto synchronous_client_lifecycle =
    std::make_shared<rclcpp::SyncParametersClient>(lifecycle_node_);
  synchronous_client_lifecycle->load_parameters(config_path);
}

void PlayMotion2HelpersTest::TearDown()
{
  node_.reset();
}

TEST_F(PlayMotion2HelpersTest, ParseMotionsKeysTest)
{
  MotionKeys keys = parse_motion_keys(node_);

  ASSERT_EQ(keys.size(), 1);
  ASSERT_EQ(keys[0], "sample");

  keys.clear();

  keys = parse_motion_keys(lifecycle_node_);

  ASSERT_EQ(keys.size(), 1);
  ASSERT_EQ(keys[0], "sample2");
}

TEST_F(PlayMotion2HelpersTest, ParseMotionInfoTest)
{
  MotionInfo info;
  ASSERT_TRUE(parse_motion_info(node_, "sample", info));

  ASSERT_EQ(info.name, "Sample");
  ASSERT_EQ(info.usage, "sample");
  ASSERT_EQ(info.description, "Sample");

  ASSERT_EQ(info.joints.size(), 2);
  ASSERT_EQ(info.joints[0], "joint1");
  ASSERT_EQ(info.joints[1], "joint2");

  ASSERT_EQ(info.positions.size(), 6);
  ASSERT_DOUBLE_EQ(info.positions[0], 0.0);
  ASSERT_DOUBLE_EQ(info.positions[1], 0.0);
  ASSERT_DOUBLE_EQ(info.positions[2], 1.0);
  ASSERT_DOUBLE_EQ(info.positions[3], 2.0);
  ASSERT_DOUBLE_EQ(info.positions[4], 2.0);
  ASSERT_DOUBLE_EQ(info.positions[5], 1.0);

  ASSERT_EQ(info.times.size(), 3);
  ASSERT_DOUBLE_EQ(info.times[0], 0.5);
  ASSERT_DOUBLE_EQ(info.times[1], 3.1);
  ASSERT_DOUBLE_EQ(info.times[2], 5.8);

  MotionInfo lifecycle_info;
  ASSERT_TRUE(parse_motion_info(lifecycle_node_, "sample2", lifecycle_info));

  ASSERT_EQ(lifecycle_info.name, "Sample2");
  ASSERT_EQ(lifecycle_info.usage, "sample2");
  ASSERT_EQ(lifecycle_info.description, "Sample2");

  ASSERT_EQ(lifecycle_info.joints.size(), 2);
  ASSERT_EQ(lifecycle_info.joints[0], "joint3");
  ASSERT_EQ(lifecycle_info.joints[1], "joint4");

  ASSERT_EQ(lifecycle_info.positions.size(), 6);
  ASSERT_DOUBLE_EQ(lifecycle_info.positions[0], 0.0);
  ASSERT_DOUBLE_EQ(lifecycle_info.positions[1], 0.0);
  ASSERT_DOUBLE_EQ(lifecycle_info.positions[2], 1.0);
  ASSERT_DOUBLE_EQ(lifecycle_info.positions[3], 2.0);
  ASSERT_DOUBLE_EQ(lifecycle_info.positions[4], 2.0);
  ASSERT_DOUBLE_EQ(lifecycle_info.positions[5], 1.0);

  ASSERT_EQ(lifecycle_info.times.size(), 3);
  ASSERT_DOUBLE_EQ(lifecycle_info.times[0], 0.5);
  ASSERT_DOUBLE_EQ(lifecycle_info.times[1], 3.1);
  ASSERT_DOUBLE_EQ(lifecycle_info.times[2], 5.8);
}

}  // namespace play_motion2
