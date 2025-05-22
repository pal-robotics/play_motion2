// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"

#include "motion_loader_test.hpp"
#include "../src/utils/motion_loader.hpp"
#include "play_motion2_msgs/msg/motion.hpp"

#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using MotionKeys = play_motion2::MotionKeys;
using MotionInfo = play_motion2::MotionInfo;

void MotionLoaderTest::SetUpTestSuite()
{
  rclcpp::init(0, nullptr);
}

void MotionLoaderTest::TearDownTestSuite()
{
  rclcpp::shutdown();
}

void MotionLoaderTest::SetUp()
{
  node_ = std::make_shared<rclcpp::Node>(
    "motion_loader_test",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true));

  // load parameters
  const auto test_path = std::filesystem::absolute(__FILE__).parent_path().string();
  const std::string config_path = test_path + "/play_motion2_config.yaml";

  auto synchronous_client =
    std::make_shared<rclcpp::SyncParametersClient>(node_);
  synchronous_client->load_parameters(config_path);

  motion_loader_ = std::make_unique<FriendMotionLoader>(
    node_->get_logger(), node_->get_node_parameters_interface());
}

void MotionLoaderTest::TearDown()
{
  node_.reset();
}

TEST_F(MotionLoaderTest, CheckGoodParamsTest)
{
  ASSERT_TRUE(motion_loader_->check_params("sample"));
}

TEST_F(MotionLoaderTest, CheckWrongPositionsParamTest)
{
  node_->undeclare_parameter("motions.sample.positions");
  ASSERT_FALSE(motion_loader_->check_params("sample"));

  // Test with wrong typed parameter
  node_->declare_parameter("motions.sample.positions", "");
  ASSERT_FALSE(motion_loader_->check_params("sample"));
}

TEST_F(MotionLoaderTest, CheckWrongJointsParamTest)
{
  node_->undeclare_parameter("motions.sample.joints");
  ASSERT_FALSE(motion_loader_->check_params("sample"));

  // Test with wrong typed parameter
  node_->declare_parameter("motions.sample.joints", 123);
  ASSERT_FALSE(motion_loader_->check_params("sample"));
}

TEST_F(MotionLoaderTest, CheckWrongTimesFromStartParamTest)
{
  // Test with no parameter
  node_->undeclare_parameter("motions.sample.times_from_start");
  ASSERT_FALSE(motion_loader_->check_params("sample"));

  // Test with wrong typed parameter
  node_->declare_parameter("motions.sample.times_from_start", "");
  ASSERT_FALSE(motion_loader_->check_params("sample"));
}

TEST_F(MotionLoaderTest, ParseMotionsKeysTest)
{
  const auto keys = motion_loader_->parse_motion_keys();

  ASSERT_EQ(keys.size(), 1u);
  ASSERT_EQ(keys[0], "sample");
}

TEST_F(MotionLoaderTest, ParseMotionInfoTest)
{
  ASSERT_TRUE(motion_loader_->parse_motion_info("sample"));
  ASSERT_FALSE(motion_loader_->parse_motion_info("undefined_motion"));

  const auto info = motion_loader_->get_motion_info("sample");

  ASSERT_EQ(info.name, "Sample");
  ASSERT_EQ(info.usage, "sample");
  ASSERT_EQ(info.description, "Sample");

  ASSERT_EQ(info.joints.size(), 2u);
  ASSERT_EQ(info.joints[0], "joint1");
  ASSERT_EQ(info.joints[1], "joint2");

  ASSERT_EQ(info.positions.size(), 6u);
  ASSERT_DOUBLE_EQ(info.positions[0], 0.0);
  ASSERT_DOUBLE_EQ(info.positions[1], 0.0);
  ASSERT_DOUBLE_EQ(info.positions[2], 1.0);
  ASSERT_DOUBLE_EQ(info.positions[3], 2.0);
  ASSERT_DOUBLE_EQ(info.positions[4], 2.0);
  ASSERT_DOUBLE_EQ(info.positions[5], 1.0);

  ASSERT_EQ(info.times.size(), 3u);
  ASSERT_DOUBLE_EQ(info.times[0], 0.5);
  ASSERT_DOUBLE_EQ(info.times[1], 3.1);
  ASSERT_DOUBLE_EQ(info.times[2], 5.8);
}

TEST_F(MotionLoaderTest, MotionExistsTest)
{
  ASSERT_TRUE(motion_loader_->parse_motion_info("sample"));
  ASSERT_TRUE(motion_loader_->exists("sample"));

  ASSERT_FALSE(motion_loader_->parse_motion_info("undefined_motion"));
  ASSERT_FALSE(motion_loader_->exists("undefined_motion"));
}

TEST_F(MotionLoaderTest, ParseAndGetMotionsTest)
{
  ASSERT_TRUE(motion_loader_->parse_motions());

  ASSERT_EQ(motion_loader_->get_motion_keys(), MotionKeys({"sample"}));
  auto & motions = motion_loader_->get_motions();
  auto & sample_motion = motion_loader_->get_motion_info("sample");

  ASSERT_EQ(motions.size(), 1u);

  ASSERT_EQ(motions.at("sample").name, sample_motion.name);
  ASSERT_EQ(motions.at("sample").usage, sample_motion.usage);
  ASSERT_EQ(motions.at("sample").description, sample_motion.description);

  ASSERT_EQ(motions.at("sample").joints.size(), sample_motion.joints.size());
  ASSERT_EQ(motions.at("sample").joints[0], sample_motion.joints[0]);
  ASSERT_EQ(motions.at("sample").joints[1], sample_motion.joints[1]);

  ASSERT_EQ(motions.at("sample").positions.size(), sample_motion.positions.size());
  ASSERT_DOUBLE_EQ(motions.at("sample").positions[0], sample_motion.positions[0]);
  ASSERT_DOUBLE_EQ(motions.at("sample").positions[1], sample_motion.positions[1]);
  ASSERT_DOUBLE_EQ(motions.at("sample").positions[2], sample_motion.positions[2]);
  ASSERT_DOUBLE_EQ(motions.at("sample").positions[3], sample_motion.positions[3]);
  ASSERT_DOUBLE_EQ(motions.at("sample").positions[4], sample_motion.positions[4]);
  ASSERT_DOUBLE_EQ(motions.at("sample").positions[5], sample_motion.positions[5]);

  ASSERT_EQ(motions.at("sample").times.size(), sample_motion.times.size());
  ASSERT_DOUBLE_EQ(motions.at("sample").times[0], sample_motion.times[0]);
  ASSERT_DOUBLE_EQ(motions.at("sample").times[1], sample_motion.times[1]);
  ASSERT_DOUBLE_EQ(motions.at("sample").times[2], sample_motion.times[2]);
}

TEST_F(MotionLoaderTest, AddWrongMotionTest)
{
  play_motion2_msgs::msg::Motion new_motion;

  // Add an empty motion
  ASSERT_FALSE(motion_loader_->add_motion(new_motion, true));

  // Set only key
  new_motion.key = "test_new_motion";
  ASSERT_FALSE(motion_loader_->add_motion(new_motion, true));

  // Set wrong parameters
  new_motion.joints = {"joint1", "joint2"};
  new_motion.positions = {0.0, 0.0, 1.0, 2.0, 2.0, 1.0};
  new_motion.times_from_start = {0.5, 3.1};
  ASSERT_FALSE(motion_loader_->add_motion(new_motion, true));
}

TEST_F(MotionLoaderTest, AddMotionTest)
{
  play_motion2_msgs::msg::Motion new_motion;
  new_motion.key = "test_new_motion";
  new_motion.joints = {"joint1", "joint2"};
  new_motion.positions = {0.0, 0.0, 1.0, 2.0, 2.0, 1.0};
  new_motion.times_from_start = {0.5, 3.1, 5.8};

  // Add a new motion and check fields
  ASSERT_TRUE(motion_loader_->add_motion(new_motion, false));
  ASSERT_TRUE(motion_loader_->exists("test_new_motion"));

  const auto & new_motion_info = motion_loader_->get_motion_info("test_new_motion");
  ASSERT_EQ(new_motion_info.joints.size(), 2u);
  ASSERT_EQ(new_motion_info.joints[0], "joint1");
  ASSERT_EQ(new_motion_info.joints[1], "joint2");
  ASSERT_EQ(new_motion_info.positions.size(), 6u);
  ASSERT_DOUBLE_EQ(new_motion_info.positions[0], 0.0);
  ASSERT_DOUBLE_EQ(new_motion_info.positions[1], 0.0);
  ASSERT_DOUBLE_EQ(new_motion_info.positions[2], 1.0);
  ASSERT_DOUBLE_EQ(new_motion_info.positions[3], 2.0);
  ASSERT_DOUBLE_EQ(new_motion_info.positions[4], 2.0);
  ASSERT_DOUBLE_EQ(new_motion_info.positions[5], 1.0);
  ASSERT_EQ(new_motion_info.times.size(), 3u);
  ASSERT_DOUBLE_EQ(new_motion_info.times[0], 0.5);
  ASSERT_DOUBLE_EQ(new_motion_info.times[1], 3.1);
  ASSERT_DOUBLE_EQ(new_motion_info.times[2], 5.8);

  // Add a change
  new_motion.times_from_start = {1.0, 2.0, 3.0};

  // overwrite flag set to false, should not update the motion
  ASSERT_FALSE(motion_loader_->add_motion(new_motion, false));
  ASSERT_EQ(new_motion_info.times.size(), 3u);
  ASSERT_DOUBLE_EQ(new_motion_info.times[0], 0.5);
  ASSERT_DOUBLE_EQ(new_motion_info.times[1], 3.1);
  ASSERT_DOUBLE_EQ(new_motion_info.times[2], 5.8);

  // overwrite flag set to true, should update the motion
  ASSERT_TRUE(motion_loader_->add_motion(new_motion, true));
  ASSERT_EQ(new_motion_info.times.size(), 3u);
  ASSERT_DOUBLE_EQ(new_motion_info.times[0], 1.0);
  ASSERT_DOUBLE_EQ(new_motion_info.times[1], 2.0);
  ASSERT_DOUBLE_EQ(new_motion_info.times[2], 3.0);
}

TEST_F(MotionLoaderTest, RemoveMotionTest)
{
  // Remove a non-existing motion
  ASSERT_FALSE(motion_loader_->remove_motion("undefined_motion"));


  // Remove an existing motion
  ASSERT_TRUE(motion_loader_->parse_motions());
  ASSERT_TRUE(motion_loader_->remove_motion("sample"));
  ASSERT_FALSE(motion_loader_->exists("sample"));
}
