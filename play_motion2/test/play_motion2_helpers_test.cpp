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
#include "rclcpp/rclcpp.hpp"

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

  // load parameters
  const auto pkg_path = ament_index_cpp::get_package_share_directory("play_motion2");
  const std::string config_path = pkg_path + "/test/config.yaml";

  auto synchronous_client =
    std::make_shared<rclcpp::SyncParametersClient>(node_);
  synchronous_client->load_parameters(config_path);
}

void PlayMotion2HelpersTest::TearDown()
{
  node_.reset();
}

TEST_F(PlayMotion2HelpersTest, ParseControllersTest)
{
  play_motion2::ControllerList controllers;
  play_motion2::parse_controllers(node_, controllers);

  ASSERT_EQ(controllers.size(), 1);
  ASSERT_EQ(controllers[0], "my_controller");
}

TEST_F(PlayMotion2HelpersTest, ParseMotionsKeysTest)
{
  play_motion2::MotionKeys keys = play_motion2::parse_motion_keys(node_);

  ASSERT_EQ(keys.size(), 1);
  ASSERT_EQ(keys[0], "sample");
}

TEST_F(PlayMotion2HelpersTest, ParseMotionInfoTest)
{
  play_motion2::MotionInfo info;
  ASSERT_TRUE(play_motion2::parse_motion_info(node_, "sample", info));

  ASSERT_EQ(info.name, "Sample");
  ASSERT_EQ(info.usage, "sample");
  ASSERT_EQ(info.description, "Sample");

  ASSERT_EQ(info.joints.size(), 2);
  ASSERT_EQ(info.joints[0], "joint1");
  ASSERT_EQ(info.joints[1], "joint2");

  ASSERT_EQ(info.trajectory.joint_names, info.joints);
  ASSERT_EQ(info.trajectory.points.size(), 3);

  ASSERT_EQ(info.trajectory.points[0].time_from_start, rclcpp::Duration::from_seconds(0.5));
  ASSERT_EQ(info.trajectory.points[0].positions.size(), info.joints.size());
  ASSERT_EQ(info.trajectory.points[0].positions[0], 0.0);
  ASSERT_EQ(info.trajectory.points[0].positions[1], 0.0);

  ASSERT_EQ(info.trajectory.points[1].time_from_start, rclcpp::Duration::from_seconds(3.1));
  ASSERT_EQ(info.trajectory.points[1].positions.size(), info.joints.size());
  ASSERT_EQ(info.trajectory.points[1].positions[0], 1.0);
  ASSERT_EQ(info.trajectory.points[1].positions[1], 2.0);

  ASSERT_EQ(info.trajectory.points[2].time_from_start, rclcpp::Duration::from_seconds(5.8));
  ASSERT_EQ(info.trajectory.points[2].positions.size(), info.joints.size());
  ASSERT_EQ(info.trajectory.points[2].positions[0], 2.0);
  ASSERT_EQ(info.trajectory.points[2].positions[1], 1.0);
}
