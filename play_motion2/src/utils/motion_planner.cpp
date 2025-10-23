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

#include <algorithm>
#include <unordered_set>
#include <queue>
#include <list>
#include <set>
#include <memory>
#include <utility>

#include "motion_planner.hpp"

#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp/version.h"

#include "std_msgs/msg/string.hpp"
namespace play_motion2
{
using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr auto kTimeout = 5s;

constexpr auto kDefaultApproachVel = 0.5;
constexpr auto kDefaultApproachMinDuration = 0.0;
constexpr auto kDefaultJointTolerance = 1e-3;

constexpr auto kMotionPlannerParametersPrefix = "motion_planner";
constexpr auto kApproachVelParam = "approach_velocity";
constexpr auto kApproachMinDurationParam = "approach_min_duration";
constexpr auto kJointToleranceParam = "joint_tolerance";
constexpr auto kDisableMotionPlanningParam = "disable_motion_planning";
constexpr auto kExcludeFromPlanningJointsParam = "exclude_from_planning_joints";
constexpr auto kPlanningGroupsParam = "planning_groups";

MotionPlanner::MotionPlanner(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: approach_vel_(kDefaultApproachVel)
  , approach_min_duration_(kDefaultApproachMinDuration)
  , joint_tolerance_(kDefaultJointTolerance)
  , no_planning_joints_()
  , planning_groups_()

  , planning_disabled_(false)

  , is_canceling_(false)

  , joint_states_sub_(nullptr)
  , joint_states_updated_(false)
  , joint_states_()
  , joint_states_mutex_()
  , joint_states_condition_()

  , node_(node)
{
  motion_planner_cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options;
  options.callback_group = motion_planner_cb_group_;

  joint_states_sub_ =
    node_->create_subscription<JointState>(
    "/joint_states", 1,
    std::bind(&MotionPlanner::joint_states_callback, this, _1), options);

#if RCLCPP_VERSION_MAJOR >= 17
  const rclcpp::QoS qos_services =
    rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1))
    .reliable()
    .durability_volatile();
#else
  const rmw_qos_profile_t qos_services = rmw_qos_profile_default;
#endif
  list_controllers_client_ = node_->create_client<ListControllers>(
    "/controller_manager/list_controllers", qos_services, motion_planner_cb_group_);

  move_group_node_ = rclcpp::Node::make_shared("_move_group_node", node_->get_name());

  check_parameters();
}

void MotionPlanner::check_parameters()
{
  // Get all motion_planner parameters
  std::map<std::string, rclcpp::Parameter> planner_params;
  node_->get_parameters(kMotionPlannerParametersPrefix, planner_params);

  // Parameters for non-planned motions
  const auto approach_vel_it = planner_params.find(kApproachVelParam);
  if (approach_vel_it != planner_params.end()) {
    if (approach_vel_it->second.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
      approach_vel_it->second.as_double() > 0.0)
    {
      approach_vel_ = approach_vel_it->second.as_double();
    } else {
      RCLCPP_WARN_STREAM(
        node_->get_logger(),
        "Param '" << kApproachVelParam << "' not set, wrong typed, negative or 0, " <<
          "using the default value: " << kDefaultApproachVel);
    }
  }

  const auto approach_min_duration_it = planner_params.find(kApproachMinDurationParam);
  if (approach_min_duration_it != planner_params.end()) {
    if (approach_min_duration_it->second.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
      approach_min_duration_it->second.as_double() >= 0.0)
    {
      approach_vel_ = approach_min_duration_it->second.as_double();
    } else {
      RCLCPP_WARN_STREAM(
        node_->get_logger(),
        "Param '" << kApproachMinDurationParam << "' not set, wrong typed or negative, " <<
          "using the default value: " << kDefaultApproachMinDuration);
    }
  }

  // Initialize motion planning capability, unless explicitly disabled
  const auto disable_planning_it = planner_params.find(kDisableMotionPlanningParam);
  if (disable_planning_it != planner_params.end()) {
    planning_disabled_ = disable_planning_it->second.as_bool();
  }

  if (planning_disabled_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Motion planning capability DISABLED. Goals requesting planning (the default) "
      "will be rejected. To disable planning in goal requests set 'skip_planning: true'");

    return;  // Skip initialization of planning-related members when all planning is disabled
  }

  const auto joint_tolerance_it = planner_params.find(kJointToleranceParam);
  if (joint_tolerance_it != planner_params.end() &&
    joint_tolerance_it->second.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
    joint_tolerance_it->second.as_double() >= 0.0)
  {
    joint_tolerance_ = joint_tolerance_it->second.as_double();
  }
  RCLCPP_DEBUG(node_->get_logger(), "Joint tolerance set to %f", joint_tolerance_);

  // Joints excluded from motion planning
  const auto no_planning_joints_it = planner_params.find(kExcludeFromPlanningJointsParam);
  if (no_planning_joints_it != planner_params.end()) {
    no_planning_joints_ = no_planning_joints_it->second.as_string_array();
  }

  // Planning group names
  const auto planning_groups_it = planner_params.find(kPlanningGroupsParam);
  if (planning_groups_it == planner_params.end() ||
    planning_groups_it->second.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
  {
    const std::string what =
      "Unspecified planning groups for computing approach trajectories. Please set the "
      "'motion_planner.planning_groups' parameter";
    throw std::runtime_error(what);
  }
  planning_groups_ = planning_groups_it->second.as_string_array();

  // Wait for robot_description and robot_description_semantic to be published
  // to avoid dying when creating MoveGroupInterface objects after 10 seconds.
  const auto wait_for_description = [&](const std::string & topic) {
      std_msgs::msg::String description;
      const auto subscription = node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).transient_local(),
        [](const std_msgs::msg::String::SharedPtr) {});

      while (!rclcpp::wait_for_message(
          description, subscription, move_group_node_->get_node_options().context(), 10s))
      {
        RCLCPP_WARN(node_->get_logger(), "Waiting for %s to be published", topic.c_str());
      }
    };

  wait_for_description("/robot_description");
  wait_for_description("/robot_description_semantic");

  for (const auto & group : planning_groups_) {
    move_groups_.emplace_back(std::make_shared<MoveGroupInterface>(move_group_node_, group));
  }
}

bool MotionPlanner::is_executable(const MotionInfo & info, const bool skip_planning)
{
  return is_executable(info.joints, skip_planning);
}

bool MotionPlanner::is_executable(const JointNames & joints, const bool skip_planning)
{
  if (planning_disabled_ && !skip_planning) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Motion planning capability is disabled, goals must not request planning. "
      "Please, set 'skip_planning: true'");
    return false;
  }

  update_controller_states_cache();

  // Get joints claimed by active controllers
  std::unordered_set<std::string> joint_names;
  for (const auto & controller : motion_controller_states_) {
    for (const auto & interface : controller.claimed_interfaces) {
      const auto joint_name = interface.substr(0, interface.find_last_of('/'));
      joint_names.insert(joint_name);
    }
  }

  bool ok = true;
  for (const auto & joint : joints) {
    // Check if joints are claimed by any active controller
    if (joint_names.find(joint) == joint_names.end()) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Joint '" << joint << "' is not claimed by any active controller");
      ok = false;
      continue;
    }
  }

  return ok;
}

void MotionPlanner::cancel_motion()
{
  is_canceling_ = true;
}

MotionInfo MotionPlanner::prepare_approach(const MotionInfo & info)
{
  const auto approach_positions = std::vector<double>(
    info.positions.begin(), info.positions.begin() + info.joints.size());

  MotionInfo approach_info = info;
  approach_info.positions = approach_positions;
  approach_info.times = {info.times[0]};

  return approach_info;
}

Result MotionPlanner::perform_motion(
  const MotionInfo & info,
  const JointTrajectory & planned_approach)
{
  // Generate controller trajectories and save the final motion time
  const auto ctrl_trajectories = generate_controller_trajectories(info, planned_approach);
  const auto final_motion_time = rclcpp::Duration(
    ctrl_trajectories.begin()->second.points.back().time_from_start).seconds();

  std::list<FollowJTGoalHandleFutureResult> futures_list;
  const auto send_result = send_trajectories(info.key, ctrl_trajectories, futures_list);
  if (send_result.state != Result::State::SUCCESS) {
    return send_result;
  }

  std::vector<std::string> controllers;
  for (const auto & traj : ctrl_trajectories) {
    controllers.push_back(traj.first);
  }

  const auto result = wait_for_results(controllers, final_motion_time, futures_list);

  return result;
}

Result MotionPlanner::execute_motion(const MotionInfo & info, const bool skip_planning)
{
  is_canceling_ = false;    // Reset canceling flag

  const auto approach_info = prepare_approach(info);

  // Check there are planned joints
  const auto planning_joints = get_planned_joints(info.joints);

  // Unplanned motion
  /// @pre planning_disabled_ = true && skip_planning = false is a combination that will
  /// not happen. Goals with that combination are rejected in is_executable.
  if (planning_joints.empty() || planning_disabled_ || skip_planning) {
    const auto approach_time =
      calculate_approach_time(approach_info.positions, approach_info.joints);

    if (approach_time < 0.0) {
      return Result(
        Result::State::ERROR,
        "Error calculating approach time, some joint has not been found in /joint_states topic");
    }

    MotionInfo unplanned_info = info;
    if (approach_time > info.times[0]) {
      for (auto & time : unplanned_info.times) {
        time = time - info.times[0] + approach_time;
      }
    }
    return perform_motion(unplanned_info, JointTrajectory());
  }

  // Planned motion
  auto move_groups = get_valid_move_groups(info.joints);
  if (move_groups.empty()) {
    return Result(Result::State::ERROR, "No valid move groups found for the given joints");
  }

  if (!needs_approach(approach_info)) {
    // If the approach trajectory is not needed and there is only one position in the motion,
    // i.e., the motion is only the approach, then the goal is already achieved.
    if (info.positions.size() == approach_info.joints.size()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Motion only consists of one position and the robot is already at the goal position.");
      return Result(Result::State::SUCCESS);
    }

    return perform_motion(info, JointTrajectory());
  }

  auto get_trajectory_from_plan = [&](const MoveGroupInterface::Plan & plan) {
    #if MOVEIT_VERSION_MINOR > 7
      return plan.trajectory;
    #else
      return plan.trajectory_;
    #endif
    };

  MoveGroupInterface::Plan approach_plan;
  for (const auto & group : move_groups) {
    approach_plan = plan_approach(group, info);
    if (!get_trajectory_from_plan(approach_plan).joint_trajectory.points.empty()) {
      break;
    }
  }

  if (get_trajectory_from_plan(approach_plan).joint_trajectory.points.empty()) {
    return Result(Result::State::ERROR, "Failed to plan approach trajectory");
  }

  return perform_motion(info, get_trajectory_from_plan(approach_plan).joint_trajectory);
}

double MotionPlanner::calculate_approach_time(
  const MotionPositions & goal_pos,
  const JointNames & joints)
{
  // wait until joint_states updated and set current positions
  std::unique_lock<std::mutex> lock(joint_states_mutex_);
  joint_states_updated_ = false;
  joint_states_condition_.wait(lock, [&] {return joint_states_updated_;});

  MotionPositions curr_pos;
  for (const auto & joint : joints) {
    if (std::find_if(
        joint_states_.begin(), joint_states_.end(),
        [&](const auto & joint_state) {return joint_state.first == joint;}) != joint_states_.end())
    {
      curr_pos.push_back(joint_states_[joint][0]);
    } else {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Joint '" << joint << "' not found in /joint_states topic");
      return -1.0;
    }
  }
  lock.unlock();

  return get_reach_time(curr_pos, goal_pos);
}

double MotionPlanner::get_reach_time(MotionPositions current_pos, MotionPositions goal_pos) const
{
  // Maximum joint displacement
  double dmax = 0.0;
  for (auto i = 0u; i < current_pos.size(); ++i) {
    const double d = std::abs(goal_pos[i] - current_pos[i]);
    if (d > dmax) {
      dmax = d;
    }
  }
  return std::max(dmax / approach_vel_, approach_min_duration_);
}

void MotionPlanner::joint_states_callback(const JointState::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(joint_states_mutex_);
  if (!joint_states_updated_) {
    joint_states_.clear();
    for (auto i = 0u; i < msg->name.size(); ++i) {
      joint_states_[msg->name[i]] = {msg->position[i], msg->velocity[i], msg->effort[i]};
    }
    joint_states_updated_ = true;
  }
  joint_states_condition_.notify_one();
}

ControllerTrajectories MotionPlanner::generate_controller_trajectories(
  const MotionInfo & info,
  const JointTrajectory & planned_approach) const
{
  ControllerTrajectories ct;
  for (const auto & controller : motion_controller_states_) {
    if (!controller.is_chained) {
      const auto trajectory = create_trajectory(controller, info, planned_approach);
      if (!trajectory.joint_names.empty()) {
        ct[controller.name] = trajectory;
      }
    }
  }


  return ct;
}

JointTrajectory MotionPlanner::create_trajectory(
  const ControllerState & controller_state,
  const MotionInfo & info,
  const JointTrajectory & planned_approach) const
{
  std::unordered_set<std::string> controller_joints;

  ControllerState chained_state = controller_state;
  for (const auto & chain : controller_state.chain_connections) {
    bool end_chain = false;
    while (!end_chain) {
      chained_state = *std::find_if(
        motion_controller_states_.begin(), motion_controller_states_.end(),
        [&](const auto & controller) {
          return controller.name == chain.name;
        });
      if (chained_state.chain_connections.empty()) {end_chain = true;}
    }
    for (const auto & interface : chained_state.claimed_interfaces) {
      std::string joint_name = interface.substr(0, interface.find_first_of('/'));
      controller_joints.insert(joint_name);
    }
  }

  if (controller_state.chain_connections.empty()) {
    for (const auto & interface : controller_state.claimed_interfaces) {
      std::string joint_name = interface.substr(0, interface.find_first_of('/'));
      controller_joints.insert(joint_name);
    }
  }

  // Create a map with joints positions
  std::map<std::string, std::vector<double>> joint_positions;
  for (const std::string & joint : controller_joints) {
    const auto iterator = std::find(info.joints.begin(), info.joints.end(), joint);
    if (iterator != info.joints.end()) {
      // get the location of the first position
      auto vector_pos = std::distance(info.joints.begin(), iterator);
      std::vector<double> positions;

      // Extract positions for a specific joint and save them
      for (auto i = 0u; i < info.times.size(); ++i) {
        positions.push_back(info.positions.at(vector_pos));
        vector_pos += info.joints.size();
      }
      joint_positions[joint] = positions;
    }
  }

  JointTrajectory jt;
  for (auto i = 0u; i < info.times.size(); ++i) {
    TrajectoryPoint jtc_point;
    const auto jtc_point_time = rclcpp::Duration::from_seconds(info.times[i]);
    jtc_point.time_from_start.sec = jtc_point_time.to_rmw_time().sec;
    jtc_point.time_from_start.nanosec = jtc_point_time.to_rmw_time().nsec;

    std::for_each(
      joint_positions.cbegin(), joint_positions.cend(),
      [&](const auto & j_pos) {
        jtc_point.positions.push_back(j_pos.second.at(i));
        jtc_point.velocities.push_back(0.0);
        jtc_point.accelerations.push_back(0.0);
      });

    jt.points.emplace_back(jtc_point);
  }

  std::for_each(
    joint_positions.cbegin(), joint_positions.cend(),
    [&](const auto & j_pos) {
      jt.joint_names.push_back(j_pos.first);
    });

  // If there is a planned approach, join it with the created trajectory
  if (!planned_approach.joint_names.empty()) {
    const auto planned_approach_time = rclcpp::Duration(
      planned_approach.points.back().time_from_start);

    // Update times with the planned approach trajectory for both planned and non-planned joints
    const auto original_approach_time = rclcpp::Duration(jt.points.front().time_from_start);
    for (auto & point : jt.points) {
      const auto new_time =
        rclcpp::Duration(point.time_from_start) - original_approach_time + planned_approach_time;
      point.time_from_start.sec = new_time.to_rmw_time().sec;
      point.time_from_start.nanosec = new_time.to_rmw_time().nsec;
    }

    if (are_all_joints_included(
        planned_approach.joint_names,
        JointNames(controller_joints.begin(), controller_joints.end())))
    {
      // Remove the original approach time and position.
      jt.points.erase(jt.points.begin());

      // Get the indexes of the respective controller joints
      std::set<unsigned int> joint_positions_indexes;
      for (const auto & joint : controller_joints) {
        const auto iterator = std::find(
          planned_approach.joint_names.begin(), planned_approach.joint_names.end(), joint);
        if (iterator != planned_approach.joint_names.end()) {
          joint_positions_indexes.insert(
            std::distance(planned_approach.joint_names.begin(), iterator));
        }
      }

      // Fill the TrajectoryPoint with the planned approach positions
      std::vector<TrajectoryPoint> planned_points;
      for (const auto & point : planned_approach.points) {
        TrajectoryPoint planned_point;
        planned_point.time_from_start = point.time_from_start;

        for (const auto & index : joint_positions_indexes) {
          planned_point.positions.push_back(point.positions[index]);
        }

        planned_points.push_back(planned_point);
      }

      // Add the planned approach points to the trajectory
      jt.points.insert(jt.points.begin(), planned_points.begin(), planned_points.end());
    }
  }

  jt.header.stamp = node_->now();
  return jt;
}

bool MotionPlanner::update_controller_states_cache()
{
  const auto controller_states = get_controller_states();

  motion_controller_states_ = filter_controller_states(
    controller_states, "active",
    "joint_trajectory_controller/JointTrajectoryController");

  RCLCPP_ERROR_EXPRESSION(
    node_->get_logger(),
    motion_controller_states_.empty(),
    "There are no active JointTrajectory controllers available");

  return !motion_controller_states_.empty();
}

ControllerStates MotionPlanner::get_controller_states() const
{
  if (!list_controllers_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Service " << list_controllers_client_->get_service_name() << " not available.");
    }
    return ControllerStates();
  }

  auto list_controllers_request = std::make_shared<ListControllers::Request>();
  auto result = list_controllers_client_->async_send_request(list_controllers_request);

  std::future_status status;
  auto start_t = node_->now();
  do {
    status = result.wait_for(0.1s);
    if (node_->now() - start_t > kTimeout) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Timeout while waiting for " << list_controllers_client_->get_service_name() <<
          " result");
      return ControllerStates();
    }
  } while(status != std::future_status::ready);

  return result.get()->controller;
}

ControllerStates MotionPlanner::filter_controller_states(
  const ControllerStates & controller_states,
  const std::string & state,
  const std::string & type) const
{
  ControllerStates filtered_controller_states;
  std::queue<std::string> chained_controllers;

  for (const auto & controller : controller_states) {
    if (controller.state == state && controller.type == type) {
      filtered_controller_states.push_back(controller);
    }

    // Add chained controllers
    if (!controller.chain_connections.empty()) {
      for (const auto & chain : controller.chain_connections) {
        chained_controllers.push(chain.name);
      }
    }
  }

  while (!chained_controllers.empty()) {
    // Get the first controller in the chain and remove it from the queue
    const auto chained_controller = chained_controllers.front();
    chained_controllers.pop();

    // Find the controller in the list of controllers
    const auto & iterator = std::find_if(
      controller_states.begin(), controller_states.end(),
      [&](const auto & controller) {return controller.name == chained_controller;});

    // If the controller is found, add it to the list of filtered controllers
    // and add its chained controllers to the list
    if (iterator != controller_states.end()) {
      filtered_controller_states.push_back(*iterator);
      if (!iterator->chain_connections.empty()) {
        for (const auto & chain : iterator->chain_connections) {
          chained_controllers.push(chain.name);
        }
      }
    } else {
      // This should never happen
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Chained controller '" << chained_controller << "' not found in the list of controllers");
      return ControllerStates();
    }
  }

  return filtered_controller_states;
}

FollowJTGoalHandleFutureResult MotionPlanner::send_trajectory(
  const std::string & controller_name,
  const JointTrajectory & trajectory)
{
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client;
  if (action_clients_.count(controller_name) != 0) {
    action_client = action_clients_[controller_name];
  } else {
    action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node_,
      "/" + controller_name + "/follow_joint_trajectory",
      motion_planner_cb_group_);
    action_clients_[controller_name] = action_client;
  }

  if (!action_client->wait_for_action_server(1s)) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "/" << controller_name <<
        "/follow_joint_trajectory action server not available after waiting");
    return {};
  }

  /// @todo fill rest of the fields ??
  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory = trajectory;

  auto goal_handle = action_client->async_send_goal(goal);

  if (!goal_handle.valid()) {
    return {};
  }
  std::future_status status;
  auto start_t = node_->now();
  do {
    status = goal_handle.wait_for(0.1s);
    if (node_->now() - start_t > kTimeout) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Timeout while waiting for " << "/" << controller_name <<
          "/follow_joint_trajectory result");
      return {};
    }
  } while(status != std::future_status::ready);

  FollowJTGoalHandleFutureResult result;
  try {
    result = action_client->async_get_result(goal_handle.get());
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    result = {};
  }

  return result;
}

Result MotionPlanner::send_trajectories(
  const std::string & motion_key,
  const ControllerTrajectories & ctrl_trajectories,
  std::list<FollowJTGoalHandleFutureResult> & futures_list)
{
  for (const auto & [controller, trajectory] : ctrl_trajectories) {
    auto jtc_future_gh = send_trajectory(controller, trajectory);
    if (!jtc_future_gh.valid()) {
      RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "Cannot perform motion '" << motion_key << "'");
      // cancel all sent goals
      cancel_all_goals();

      return Result(
        Result::State::ERROR,
        "Motion " + motion_key + " aborted. Cannot send goal to " + controller);
    }
    futures_list.push_back(std::move(jtc_future_gh));
  }

  return Result(Result::State::SUCCESS);
}

Result MotionPlanner::wait_for_results(
  const std::vector<std::string> & motion_controllers,
  const double motion_time,
  std::list<FollowJTGoalHandleFutureResult> & futures_list)
{
  Result result;
  bool failed = false;

  if (is_canceling_) {
    return Result(Result::State::CANCELED, "Motion canceled");
  }

  // Spin all futures and remove them when succeeded.
  // If one fails, set failed to true and returns false
  const auto successful_jt = [&](FollowJTGoalHandleFutureResult & future) {
      std::future_status status = future.wait_for(0.1s);
      if (status == std::future_status::ready) {
        if (future.get().code == rclcpp_action::ResultCode::SUCCEEDED) {
          return true;
        } else {
          cancel_all_goals();
          failed = true;
          result = Result(Result::State::ERROR, "Joint Trajectory failed");
          RCLCPP_ERROR_STREAM(node_->get_logger(), result.error);
        }
      }
      return false;
    };

  // finish if failed, motions finished or timeout
  const double TIMEOUT = motion_time * 2.0 + 1.0;
  const rclcpp::Time init_time = node_->now();
  bool on_time = true;
  do {
    futures_list.erase(
      std::remove_if(futures_list.begin(), futures_list.end(), successful_jt),
      futures_list.end());
    on_time = (node_->now() - init_time).seconds() < TIMEOUT;

    auto current_states = filter_controller_states(
      get_controller_states(), "active", "joint_trajectory_controller/JointTrajectoryController");

    // If any controller changes, check if it is used in the motion.
    // If so, cancel all goals and return an error.
    if (current_states != motion_controller_states_) {
      std::vector<std::string> current_controllers;
      for (const auto & controller : current_states) {
        current_controllers.push_back(controller.name);
      }

      for (const auto & controller : motion_controllers) {
        if (std::find(current_controllers.begin(), current_controllers.end(), controller) ==
          current_controllers.end())
        {
          cancel_all_goals();
          failed = true;
          result = Result(
            Result::State::ERROR,
            "Controller '" + controller + "' has been deactivated while executing the motion");
          RCLCPP_ERROR_STREAM(node_->get_logger(), result.error);
        }
      }
    }

    if (is_canceling_) {
      cancel_all_goals();
      futures_list.clear();
      return Result(Result::State::CANCELED, "Motion canceled");
    }
  } while (!failed && !futures_list.empty() && on_time);

  if (!on_time) {
    cancel_all_goals();
    result = Result(Result::State::ERROR, "Timeout exceeded while waiting for results");
    RCLCPP_ERROR_STREAM(node_->get_logger(), result.error);
  } else if (!failed) {   // All goals succeeded
    result = Result(Result::State::SUCCESS);
  }

  return result;
}

std::vector<MoveGroupInterfacePtr>
MotionPlanner::get_valid_move_groups(const JointNames & joints) const
{
  const auto planning_joints = get_planned_joints(joints);

  std::vector<MoveGroupInterfacePtr> valid_move_groups;
  for (const auto & move_group : move_groups_) {
    const auto group_joints = move_group->getJointNames();
    if (are_all_joints_included(group_joints, planning_joints)) {
      valid_move_groups.push_back(move_group);
    }
  }
  return valid_move_groups;
}

JointNames MotionPlanner::get_planned_joints(const JointNames & joints) const
{
  auto planning_joints = joints;
  for (const auto & joint : no_planning_joints_) {
    planning_joints.erase(
      std::remove(planning_joints.begin(), planning_joints.end(), joint),
      planning_joints.end());
  }
  return planning_joints;
}

MoveGroupInterface::Plan MotionPlanner::plan_approach(
  MoveGroupInterfacePtr group,
  const MotionInfo & approach_info)
{
  group->setStartStateToCurrentState();
  group->setGoalTolerance(joint_tolerance_);

  /// Without this, the movement is very slow
  /// @todo Investigate if we can do it in moveit2 configuration
  group->setMaxVelocityScalingFactor(1.0);

  /// @pre sizes of joints and positions are the same
  for (auto i = 0u; i < approach_info.joints.size(); ++i) {
    if (std::find(
        no_planning_joints_.begin(), no_planning_joints_.end(),
        approach_info.joints[i]) == no_planning_joints_.end())
    {
      if (!group->setJointValueTarget(approach_info.joints[i], approach_info.positions[i])) {
        RCLCPP_ERROR_STREAM(
          node_->get_logger(), "Failed attempt to set planning goal for joint '" <<
            approach_info.joints[i] << "' on group '" << group->getName() << "'.");
        return MoveGroupInterface::Plan();
      }
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Failed to plan for group '" << group->getName() << "'.");
    return MoveGroupInterface::Plan();
  }

  return plan;
}

bool MotionPlanner::are_all_joints_included(
  const JointNames & full_joint_names,
  const JointNames & partial_joint_names) const
{
  for (const auto & joint : partial_joint_names) {
    if (std::find(
        full_joint_names.begin(), full_joint_names.end(), joint) ==
      full_joint_names.end())
    {
      return false;
    }
  }
  return true;
}

bool MotionPlanner::needs_approach(const MotionInfo & approach_info)
{
  // Wait until joint_states_ updated and set current positions
  std::unique_lock<std::mutex> lock(joint_states_mutex_);
  joint_states_updated_ = false;
  joint_states_condition_.wait(lock, [&] {return joint_states_updated_;});

  for (const auto & joint : approach_info.joints) {
    {
      const auto joint_pos = std::distance(
        approach_info.joints.begin(),
        std::find(approach_info.joints.begin(), approach_info.joints.end(), joint));
      const auto goal_pos = approach_info.positions[joint_pos];
      const auto current_pos = joint_states_[joint][0];

      if (std::abs(current_pos - goal_pos) > joint_tolerance_) {
        return true;
      }
    }
  }
  return false;
}

void MotionPlanner::cancel_all_goals()
{
  // cancel all sent goals
  for (const auto & client : action_clients_) {
    client.second->async_cancel_all_goals();
  }
}

}     // namespace play_motion2
