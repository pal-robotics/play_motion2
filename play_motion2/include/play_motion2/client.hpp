// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

#include <string>
#include <vector>

#include "play_motion2/types.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/action/play_motion2_raw.hpp"
#include "play_motion2_msgs/msg/motion.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"
#include "play_motion2_msgs/srv/is_motion_ready.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_msgs/srv/remove_motion.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp/node.hpp"

#ifndef PLAY_MOTION2__CLIENT_HPP_
#define PLAY_MOTION2__CLIENT_HPP_

namespace play_motion2
{
/**
   * @class PlayMotion2Client
   * @brief A client Node for the PlayMotion2 action server.
   * This node provides methods to run motions, check if a motion is running,
   * and manage motions (add, remove, list, check readiness).
   */
class PlayMotion2Client : public rclcpp::Node
{
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
  using PlayMotion2Raw = play_motion2_msgs::action::PlayMotion2Raw;
  using MotionMsg = play_motion2_msgs::msg::Motion;

  using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;
  using IsMotionReady = play_motion2_msgs::srv::IsMotionReady;
  using ListMotions = play_motion2_msgs::srv::ListMotions;

  using AddMotion = play_motion2_msgs::srv::AddMotion;
  using RemoveMotion = play_motion2_msgs::srv::RemoveMotion;

public:
  /**
   * @brief Constructor for the PlayMotion2Client.
   * @param name The name of the node (defaults to "play_motion2_client").
   * @param options Node options for configuration.
   */
  explicit PlayMotion2Client(
    const std::string & name = "play_motion2_client",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the PlayMotion2Client.
   */
  ~PlayMotion2Client();

  /**
   * @brief Run a motion by its name.
   * @param motion_name The name of the motion to run.
   * @param skip_planning If true, skip the planning phase.
   * @param motion_timeout Optional. The timeout for the motion execution (defaults to 120).
   * @return True if the motion was successfully started, false otherwise.
   */
  bool run_motion(
    const std::string & motion_name,
    const bool skip_planning,
    const std::chrono::seconds & motion_timeout = std::chrono::seconds(120));

  /**
   * @brief Run a motion asynchronously by its name.
   * @param motion_name The name of the motion to run.
   * @param skip_planning If true, skip the planning phase.
   * @return True if the motion was successfully started, false otherwise.
   */
  bool run_motion_async(
    const std::string & motion_name,
    const bool skip_planning);

  /**
 * @brief Run a non-stored motion.
 * @param motion The motion to run.
 * @param skip_planning If true, skip the planning phase.
 * @param motion_timeout Optional. The timeout for the motion execution (defaults to 120).
 * @return True if the motion was successfully started, false otherwise.
 */
  bool run_motion(
    const play_motion2_msgs::msg::Motion & motion,
    const bool skip_planning,
    const std::chrono::seconds & motion_timeout = std::chrono::seconds(120));

  /**
   * @brief Run a non-stored motion asynchronously.
   * @param motion The motion to run.
   * @param skip_planning If true, skip the planning phase.
   * @return True if the motion was successfully started, false otherwise.
   */
  bool run_motion_async(
    const play_motion2_msgs::msg::Motion & motion,
    const bool skip_planning);

  /**
   * @brief Check if there is a motion currently running on the system.
   * @return True if a motion is running, false otherwise.
   */
  bool is_running_motion() const;
  /**
   * @brief Check if the last motion execution was successful.
   * @return True if the last motion succeeded, false otherwise.
   */
  bool last_succeeded() const;

  /**
   * @brief List all available motions.
   * @return A vector of motion keys (names) available in the system.
   */
  std::vector<std::string> list_motions();
  /**
   * @brief Check if a specific motion is ready to be executed.
   * @param motion_key The key (name) of the motion to check.
   * @return True if the motion is ready, false otherwise.
   */
  bool is_motion_ready(const std::string & motion_key);
  /**
   * @brief Get detailed information about a specific motion.
   * @param motion_key The key (name) of the motion to retrieve information for.
   * @return A MotionInfo object containing details about the motion.
   */
  MotionInfo get_motion_info(const std::string & motion_key);

  /**
   * @brief Add a new motion to the system.
   * @param motion_msg The MotionMsg object containing the motion details.
   * @param overwrite If true, overwrite an existing motion with the same key.
   * @return True if the motion was successfully added, false otherwise.
   */
  bool add_motion(const MotionMsg & motion_msg, const bool overwrite);
  /**
   * @brief Remove a motion from the system.
   * @param motion_key The key (name) of the motion to remove.
   * @return True if the motion was successfully removed, false otherwise.
   */
  bool remove_motion(const std::string & motion_key);

private:
  const rclcpp_action::ClientGoalHandle<PlayMotion2>::SharedPtr
  send_goal(const std::string & motion_name, const bool skip_planning);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<PlayMotion2>::WrappedResult & result);

  const rclcpp_action::ClientGoalHandle<PlayMotion2Raw>::SharedPtr
  send_goal_raw(const play_motion2_msgs::msg::Motion & motion, const bool skip_planning);
  void result_callback_raw(
    const rclcpp_action::ClientGoalHandle<PlayMotion2Raw>::WrappedResult & result);

private:
  std::atomic_bool running_motion_;
  bool motion_succeeded_;

  rclcpp_action::Client<PlayMotion2>::SharedPtr play_motion2_client_;
  rclcpp_action::Client<PlayMotion2Raw>::SharedPtr play_motion2_client_raw_;

  rclcpp::Client<GetMotionInfo>::SharedPtr get_motion_info_client_;
  rclcpp::Client<IsMotionReady>::SharedPtr is_motion_ready_client_;
  rclcpp::Client<ListMotions>::SharedPtr list_motions_client_;

  rclcpp::Client<AddMotion>::SharedPtr add_motion_client_;
  rclcpp::Client<RemoveMotion>::SharedPtr remove_motion_client_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__CLIENT_HPP_
