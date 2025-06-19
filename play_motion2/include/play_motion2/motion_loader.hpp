// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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

#ifndef PLAY_MOTION2__MOTION_LOADER_HPP_
#define PLAY_MOTION2__MOTION_LOADER_HPP_

#include <string>

#include "play_motion2/types.hpp"
#include "play_motion2_msgs/msg/motion.hpp"

namespace play_motion2
{

class MotionLoaderBase
{
protected:
  using MotionMsg = play_motion2_msgs::msg::Motion;

public:
  MotionLoaderBase() = default;
  virtual ~MotionLoaderBase() = default;

  virtual bool parse_motions() = 0;
  virtual bool exists(const std::string & motion_key) const = 0;

  virtual const MotionKeys & get_motion_keys() const = 0;
  virtual const MotionInfo & get_motion_info(const std::string & motion_key) const = 0;
  virtual const MotionsMap & get_motions() const = 0;

  virtual bool add_motion(const MotionMsg & motion_msg, const bool overwrite) = 0;
  virtual bool remove_motion(const std::string & motion_key) = 0;
};

}  // namespace play_motion2

#endif  // PLAY_MOTION2__MOTION_LOADER_HPP_
