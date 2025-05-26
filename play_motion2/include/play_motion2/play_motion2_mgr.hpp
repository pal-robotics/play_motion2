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

#ifndef PLAY_MOTION2__PLAY_MOTION2_MGR_HPP_
#define PLAY_MOTION2__PLAY_MOTION2_MGR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/action/play_motion2_raw.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/is_joint_list_ready.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_msgs/srv/remove_motion.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace play_motion2
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;
using IsMotionReady = play_motion2_msgs::srv::IsMotionReady;
using IsJointListReady = play_motion2_msgs::srv::IsJointListReady;
using ListMotions = play_motion2_msgs::srv::ListMotions;
using AddMotion = play_motion2_msgs::srv::AddMotion;
using RemoveMotion = play_motion2_msgs::srv::RemoveMotion;

/**
 * @class PlayMotion2MgrBase
 * @brief Base class for managing PlayMotion2 actions and services.
 * This class provides the basic structure for managing motions, including adding,
 * removing, listing, and checking the readiness of motions. It also provides lifecycle management
 * for the node, allowing it to be configured, activated, deactivated, cleaned up, and shut down.
 * It is intended to be extended by specific implementations that provide the actual
 * functionality for managing motions.
 */
class PlayMotion2MgrBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  // Callback group for the action client to /play_motion2/raw
  rclcpp::CallbackGroup::SharedPtr ac_group_;
  // Callback group for the action server
  rclcpp::CallbackGroup::SharedPtr as_group_;
  // Callback group for the is_joint_list_ready service (client)
  rclcpp::CallbackGroup::SharedPtr srv_c_group_;
  // Callback group for the add/list/remove motion and is_motion_ready services (server)
  rclcpp::CallbackGroup::SharedPtr srv_s_group_;

  explicit PlayMotion2MgrBase(const rclcpp::NodeOptions & options);
  ~PlayMotion2MgrBase() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Configure the manager.
   * This method should be implemented by derived classes to perform any necessary
   * configuration when the node is activated. It will be called during the configure transition
   * of the lifecycle.
   * @return true if configuration was successful, false otherwise.
   */
  virtual bool configureMgr() = 0;
  /**
   * @brief Cleanup the manager.
   * This method should be implemented by derived classes to perform any necessary
   * cleanup when the node is deactivated. It will be called during the cleanup transition
   * of the lifecycle.
   */
  virtual void cleanupMgr() = 0;
  /**
   * @brief Get the keys of all available motions.
   * This method should be implemented by derived classes to return a list of motion keys.
   * @return A vector of strings containing the motion keys.
   */
  virtual const std::vector<std::string> getMotionKeys() const = 0;
  /**
   * @brief Add a motion to the manager.
   * This method should be implemented by derived classes to add a motion.
   * @param motion_msg The motion message to add.
   * @param overwrite If true, overwrite an existing motion with the same key.
   * @return true if the motion was added successfully, false otherwise.
   */
  virtual bool addMotion(
    const play_motion2_msgs::msg::Motion & motion_msg,
    const bool overwrite) = 0;
  /**
   * @brief Remove a motion from the manager.
   * This method should be implemented by derived classes to remove a motion by its key.
   * @param motion_key The key of the motion to remove.
   * @return true if the motion was removed successfully, false otherwise.
   */
  virtual bool removeMotion(const std::string & motion_key) = 0;
  /**
   * @brief Check if a motion exists in the manager.
   * This method should be implemented by derived classes to check if a motion with the given name exists.
   * @param motion_name The name of the motion to check.
   * @return true if the motion exists, false otherwise.
   */
  virtual bool motionExists(const std::string & motion_name) const = 0;
  /**
   * @brief Load a motion by its name.
   * This method should be implemented by derived classes to load a motion from storage or memory.
   * @param motion_name The name of the motion to load.
   * @return The loaded motion message.
   */
  virtual play_motion2_msgs::msg::Motion loadMotion(const std::string & motion_name) const = 0;

private:
  rclcpp::Service<GetMotionInfo>::SharedPtr get_motion_info_service_;
  rclcpp::Service<IsMotionReady>::SharedPtr is_motion_ready_service_;
  rclcpp::Service<ListMotions>::SharedPtr list_motions_service_;
  rclcpp::Client<IsJointListReady>::SharedPtr is_joint_list_ready_client_;

  rclcpp::Service<AddMotion>::SharedPtr add_motion_service_;
  rclcpp::Service<RemoveMotion>::SharedPtr remove_motion_service_;

  rclcpp_action::Server<play_motion2_msgs::action::PlayMotion2>::SharedPtr pm2_action_;
  rclcpp_action::Client<play_motion2_msgs::action::PlayMotion2Raw>::SharedPtr pm2_raw_action_;

  rclcpp_action::ClientGoalHandle<play_motion2_msgs::action::PlayMotion2Raw>::SharedPtr
    current_goal_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<play_motion2_msgs::action::PlayMotion2>>
  current_goal_handle_;

  void listMotionsCallback(
    ListMotions::Request::ConstSharedPtr request,
    ListMotions::Response::SharedPtr response) const;

  void isMotionReadyCallback(
    IsMotionReady::Request::ConstSharedPtr request,
    IsMotionReady::Response::SharedPtr response);

  void getMotionInfoCallback(
    GetMotionInfo::Request::ConstSharedPtr request,
    GetMotionInfo::Response::SharedPtr response) const;

  void addMotionCallback(
    AddMotion::Request::ConstSharedPtr request,
    AddMotion::Response::SharedPtr response);

  void removeMotionCallback(
    RemoveMotion::Request::ConstSharedPtr request,
    RemoveMotion::Response::SharedPtr response);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const play_motion2_msgs::action::PlayMotion2::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      play_motion2_msgs::action::PlayMotion2>> goal_handle)
  const;

  void handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      play_motion2_msgs::action::PlayMotion2>> goal_handle);
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__PLAY_MOTION2_MGR_HPP_
