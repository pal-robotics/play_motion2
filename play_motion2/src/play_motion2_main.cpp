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

#include "rclcpp/executors.hpp"

#include "play_motion2/play_motion2_executor.hpp"
#include "play_motion2/play_motion2_mgr.hpp"

#include "utils/motion_loader.hpp"

namespace play_motion2
{
class PlayMotion2PublicMgr : public PlayMotion2MgrBase
{
public:
  explicit PlayMotion2PublicMgr(const rclcpp::NodeOptions & options)
  : PlayMotion2MgrBase(options)
  {
  }

  ~PlayMotion2PublicMgr() override
  {
    // Destructor implementation
  }


  virtual bool configureMgr()
  {
    motion_loader_ = std::make_unique<MotionLoader>(get_logger(), get_node_parameters_interface());
    return motion_loader_->parse_motions();
  }

  virtual void cleanupMgr()
  {
    motion_loader_.reset();
  }

  virtual const std::vector<std::string> getMotionKeys() const
  {
    return motion_loader_->get_motion_keys();
  }

  virtual bool addMotion(
    const play_motion2_msgs::msg::Motion & motion_msg,
    const bool overwrite)
  {
    return motion_loader_->add_motion(motion_msg, overwrite);
  }

  virtual bool removeMotion(const std::string & motion_key)
  {
    return motion_loader_->remove_motion(motion_key);
  }

  virtual bool motionExists(const std::string & motion_name) const
  {
    return motion_loader_->exists(motion_name);
  }

  virtual play_motion2_msgs::msg::Motion loadMotion(const std::string & motion_name) const
  {
    play_motion2_msgs::msg::Motion motion_msg;

    const auto motion_info = motion_loader_->get_motion_info(motion_name);
    motion_msg.key = motion_info.key;
    motion_msg.name = motion_info.name;
    motion_msg.usage = motion_info.usage;
    motion_msg.description = motion_info.description;
    motion_msg.joints = motion_info.joints;
    motion_msg.positions = motion_info.positions;
    motion_msg.times_from_start = motion_info.times;

    return motion_msg;
  }

private:
  std::unique_ptr<MotionLoader> motion_loader_;
};
}  // namespace play_motion2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options =
    rclcpp::NodeOptions().allow_undeclared_parameters(true).
    automatically_declare_parameters_from_overrides(
    true);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto mgrNode = std::make_shared<play_motion2::PlayMotion2PublicMgr>(options);
  executor.add_node(mgrNode->get_node_base_interface());

  auto executorNode = std::make_shared<play_motion2::PlayMotion2Executor>(options);
  executor.add_node(executorNode->get_node_base_interface());

  executorNode->configure();
  mgrNode->configure();
  executor.spin_some();

  executorNode->activate();
  mgrNode->activate();
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
