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

#ifndef PLAY_MOTION2_HELPERS_TEST_HPP_
#define PLAY_MOTION2_HELPERS_TEST_HPP_

#include <memory>

#include "gtest/gtest.h"

namespace rclcpp_lifecycle
{
class LifecycleNode;
}

namespace play_motion2
{

class PlayMotion2HelpersTest : public ::testing::Test
{
public:
  PlayMotion2HelpersTest() = default;
  ~PlayMotion2HelpersTest() = default;

  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp();
  void TearDown();

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2_HELPERS_TEST_HPP_
