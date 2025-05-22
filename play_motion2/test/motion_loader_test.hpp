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

#ifndef MOTION_LOADER_TEST_HPP_
#define MOTION_LOADER_TEST_HPP_

#include <memory>

#include "gtest/gtest.h"

#include "../src/utils/motion_loader.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class FriendMotionLoader : public play_motion2::MotionLoader
{
  using NodeParametersInterfaceSharedPtr =
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr;

  friend class MotionLoaderTest;

public:
  FriendMotionLoader(
    const rclcpp::Logger logger,
    const NodeParametersInterfaceSharedPtr parameters_interface)
  : play_motion2::MotionLoader(logger, parameters_interface) {}

  virtual ~FriendMotionLoader() = default;

  FRIEND_TEST(MotionLoaderTest, CheckGoodParamsTest);
  FRIEND_TEST(MotionLoaderTest, CheckWrongPositionsParamTest);
  FRIEND_TEST(MotionLoaderTest, CheckWrongJointsParamTest);
  FRIEND_TEST(MotionLoaderTest, CheckWrongTimesFromStartParamTest);
  FRIEND_TEST(MotionLoaderTest, ParseMotionsKeysTest);
  FRIEND_TEST(MotionLoaderTest, ParseMotionInfoTest);
  FRIEND_TEST(MotionLoaderTest, MotionExistsTest);
  FRIEND_TEST(MotionLoaderTest, ParseAndGetMotionsTest);
};

class MotionLoaderTest : public ::testing::Test
{
public:
  MotionLoaderTest() = default;
  ~MotionLoaderTest() = default;

  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp();
  void TearDown();

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FriendMotionLoader> motion_loader_;
};

#endif  // MOTION_LOADER_TEST_HPP_
