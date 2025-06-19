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

#ifndef UTILS__MOTION_LOADER_HPP_
#define UTILS__MOTION_LOADER_HPP_

#include <string>

#include "play_motion2/motion_loader.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

namespace play_motion2
{
using NodeParametersInterfaceSharedPtr =
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr;

class MotionLoader : public MotionLoaderBase
{
public:
  MotionLoader(
    const rclcpp::Logger logger,
    const NodeParametersInterfaceSharedPtr parameters_interface);
  virtual ~MotionLoader() = default;

  bool parse_motions() override;
  bool exists(const std::string & motion_key) const override;

  const MotionKeys & get_motion_keys() const override;
  const MotionInfo & get_motion_info(const std::string & motion_key) const override;
  const MotionsMap & get_motions() const override;

  bool add_motion(const MotionMsg & motion_msg, const bool overwrite) override;
  bool remove_motion(const std::string & motion_key) override;

protected:
  MotionKeys parse_motion_keys() const;
  bool check_params(const std::string & motion_key) const;
  bool parse_motion_info(const std::string & motion_key);

private:
  rclcpp::Logger logger_;
  NodeParametersInterfaceSharedPtr params_interface_;

  MotionKeys motion_keys_;
  MotionsMap motions_;
};

}  // namespace play_motion2

#endif  // UTILS__MOTION_LOADER_HPP_
