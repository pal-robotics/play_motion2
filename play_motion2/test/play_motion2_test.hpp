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

#ifndef PLAY_MOTION2_TEST_HPP_
#define PLAY_MOTION2_TEST_HPP_

#include <memory>

#include "gtest/gtest.h"
#include "play_motion2/play_motion2.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "rclcpp/rclcpp.hpp"

class PlayMotion2Test : public ::testing::Test
{
public:
  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp();
  void TearDown();

protected:
  std::shared_ptr<play_motion2::PlayMotion2> play_motion2_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread runner_;
};

#endif  // PLAY_MOTION2_TEST_HPP_
