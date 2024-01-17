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

#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "play_motion2/approach_planner.hpp"
#include "play_motion2/motion_loader.hpp"
#include "play_motion2/play_motion2.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace play_motion2
{
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

constexpr auto kTimeout = 5s;

PlayMotion2::PlayMotion2()
: LifecycleNode("play_motion2",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
  client_cb_group_(),
  is_motion_ready_service_(),
  list_motions_service_(),
  pm2_action_(),
  list_controllers_client_(),
  action_clients_(),
  motion_controller_states_(),
  motion_executor_(),
  is_busy_(false)
{
}

PlayMotion2::~PlayMotion2()
{
  // wait if a motion is being executed until it finishes
  if (motion_executor_.joinable()) {
    motion_executor_.join();
  }
}

CallbackReturn PlayMotion2::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  motion_loader_ = std::make_unique<MotionLoader>(get_logger(), get_node_parameters_interface());
  const bool ok = motion_loader_->parse_motions();

  approach_planner_ = std::make_unique<ApproachPlanner>(shared_from_this());

  RCLCPP_ERROR_EXPRESSION(get_logger(), !ok, "Failed to initialize Play Motion 2");

  return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn PlayMotion2::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  list_motions_service_ = create_service<ListMotions>(
    "play_motion2/list_motions",
    std::bind(&PlayMotion2::list_motions_callback, this, _1, _2));

  is_motion_ready_service_ = create_service<IsMotionReady>(
    "play_motion2/is_motion_ready",
    std::bind(&PlayMotion2::is_motion_ready_callback, this, _1, _2));

  pm2_action_ = rclcpp_action::create_server<PlayMotion2Action>(
    shared_from_this(), "play_motion2",
    std::bind(&PlayMotion2::handle_goal, this, _1, _2),
    std::bind(&PlayMotion2::handle_cancel, this, _1),
    std::bind(&PlayMotion2::handle_accepted, this, _1)
  );

  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  list_controllers_client_ = create_client<ListControllers>(
    "/controller_manager/list_controllers", rmw_qos_profile_default, client_cb_group_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo reject when a motion is being executed ?

  list_motions_service_.reset();
  is_motion_ready_service_.reset();
  pm2_action_.reset();
  client_cb_group_.reset();
  list_controllers_client_.reset();
  is_busy_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  motion_loader_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  /// @todo cancel all goals
  return CallbackReturn::SUCCESS;
}

CallbackReturn PlayMotion2::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void PlayMotion2::list_motions_callback(
  ListMotions::Request::ConstSharedPtr /*request*/,
  ListMotions::Response::SharedPtr response) const
{
  response->motion_keys = motion_loader_->get_motion_keys();
}

void PlayMotion2::is_motion_ready_callback(
  IsMotionReady::Request::ConstSharedPtr request,
  IsMotionReady::Response::SharedPtr response)
{
  response->is_ready = update_controller_states_cache() && is_executable(request->motion_key);
}

rclcpp_action::GoalResponse PlayMotion2::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlayMotion2Action::Goal> goal)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received goal request: motion '" << goal->motion_name << "'");

  if (!update_controller_states_cache() || is_busy_ || !is_executable(goal->motion_name)) {
    RCLCPP_ERROR_EXPRESSION(get_logger(), is_busy_, "PlayMotion2 is busy");
    RCLCPP_ERROR_STREAM(get_logger(), "Motion '" << goal->motion_name << "' cannot be performed");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (motion_executor_.joinable()) {
    motion_executor_.join();
  }
  is_busy_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlayMotion2::handle_cancel(
  const std::shared_ptr<GoalHandlePM2> goal_handle) const
{
  RCLCPP_INFO_STREAM(get_logger(), "Cancelling motion " << goal_handle->get_goal()->motion_name);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlayMotion2::handle_accepted(const std::shared_ptr<GoalHandlePM2> goal_handle)
{
  motion_executor_ = std::thread{std::bind(&PlayMotion2::execute_motion, this, _1), goal_handle};
}

void PlayMotion2::execute_motion(const std::shared_ptr<GoalHandlePM2> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto & motion = motion_loader_->get_motion_info(goal->motion_name);

  const auto approach_time =
    approach_planner_->calculate_approach_time(motion);
  auto extra_time = 0.0;
  if (motion.times[0] < approach_time) {
    extra_time = approach_time - motion.times[0];
  }

  const auto ctrl_trajectories = generate_controller_trajectories(goal->motion_name, approach_time);

  std::list<FollowJTGoalHandleFutureResult> futures_list;
  for (const auto & [controller, trajectory] : ctrl_trajectories) {
    auto jtc_future_gh = send_trajectory(controller, trajectory);
    if (!jtc_future_gh.valid()) {
      RCLCPP_INFO_STREAM(get_logger(), "Cannot perform motion '" << goal->motion_name << "'");
      // cancel all sent goals
      for (const auto & client : action_clients_) {
        client.second->async_cancel_all_goals();
      }

      auto result = std::make_shared<PlayMotion2Action::Result>();
      result->success = false;
      result->error = "Motion " + goal->motion_name + " aborted. Cannot send goal to " + controller;
      goal_handle->abort(result);
      is_busy_ = false;
      return;
    }
    futures_list.push_back(std::move(jtc_future_gh));
  }

  /// @todo send feedback
  auto result = wait_for_results(
    goal_handle, futures_list,
    motion.times.back() + extra_time);

  if (goal_handle->is_active()) {
    if (!result->success) {
      RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion_name << "' failed");
      goal_handle->abort(result);
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Motion '" << goal->motion_name << "' completed");
      goal_handle->succeed(result);
    }
  }
  is_busy_ = false;
}

bool PlayMotion2::update_controller_states_cache()
{
  if (is_busy_) {
    return false;
  }

  const auto controller_states = get_controller_states();

  motion_controller_states_ = filter_controller_states(
    controller_states, "active",
    "joint_trajectory_controller/JointTrajectoryController");

  RCLCPP_ERROR_EXPRESSION(
    get_logger(),
    motion_controller_states_.empty(),
    "There are no active JointTrajectory controllers available");

  return !motion_controller_states_.empty();
}

bool PlayMotion2::is_executable(const std::string & motion_key) const
{
  return motion_loader_->exists(motion_key) &&
         check_joints_and_controllers(motion_key);
}

ControllerStates PlayMotion2::get_controller_states() const
{
  if (!list_controllers_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Service " << list_controllers_client_->get_service_name() << " not available.");
    }
    return ControllerStates();
  }

  auto list_controllers_request = std::make_shared<ListControllers::Request>();
  auto result = list_controllers_client_->async_send_request(list_controllers_request);

  std::future_status status;
  auto start_t = now();
  do {
    status = result.wait_for(0.1s);
    if (now() - start_t > kTimeout) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Timeout while waiting for " << list_controllers_client_->get_service_name() <<
          " result");
      return ControllerStates();
    }
  } while(status != std::future_status::ready);

  return result.get()->controller;
}


ControllerStates PlayMotion2::filter_controller_states(
  const ControllerStates & controller_states,
  const std::string & state,
  const std::string & type) const
{
  ControllerStates filtered_controller_states;

  for (const auto & controller : controller_states) {
    if (controller.state == state && controller.type == type) {
      filtered_controller_states.push_back(controller);
    }
  }

  return filtered_controller_states;
}

bool PlayMotion2::check_joints_and_controllers(const std::string & motion_key) const
{
  // get joints claimed by active controllers
  std::unordered_set<std::string> joint_names;
  for (const auto & controller : motion_controller_states_) {
    for (const auto & interface : controller.claimed_interfaces) {
      const auto joint_name = interface.substr(0, interface.find_first_of('/'));
      joint_names.insert(joint_name);
    }
  }

  bool ok = true;
  for (const auto & joint : motion_loader_->get_motion_info(motion_key).joints) {
    // check joints are claimed by any active controller
    if (joint_names.find(joint) == joint_names.end()) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Joint '" << joint << "' is not claimed by any active controller");
      ok = false;
      continue;
    }
  }

  return ok;
}

JTMsg PlayMotion2::create_trajectory(
  const ControllerState & controller_state,
  const std::string & motion_key,
  const double extra_time) const
{
  std::unordered_set<std::string> controller_joints;
  for (const auto & interface : controller_state.claimed_interfaces) {
    std::string joint_name = interface.substr(0, interface.find_first_of('/'));
    controller_joints.insert(joint_name);
  }

  // create a map with joints positions
  const auto & motion_info = motion_loader_->get_motion_info(motion_key);
  std::map<std::string, std::vector<double>> joint_positions;
  for (const std::string & joint : controller_joints) {
    const auto iterator = std::find(motion_info.joints.begin(), motion_info.joints.end(), joint);
    if (iterator != motion_info.joints.end()) {
      // get the location of the first position
      unsigned int vector_pos = std::distance(motion_info.joints.begin(), iterator);
      std::vector<double> positions;

      // extract positions for a specific joint and save them
      for (unsigned int i = 0; i < motion_info.times.size(); i++) {
        positions.push_back(motion_info.positions.at(vector_pos));
        vector_pos += motion_info.joints.size();
      }
      joint_positions[joint] = positions;
    }
  }

  JTMsg jt_msg;
  for (unsigned int i = 0; i < motion_info.times.size(); i++) {
    TrajectoryPoint jtc_point;
    const auto jtc_point_time = rclcpp::Duration::from_seconds(motion_info.times[i] + extra_time);
    jtc_point.time_from_start.sec = jtc_point_time.to_rmw_time().sec;
    jtc_point.time_from_start.nanosec = jtc_point_time.to_rmw_time().nsec;

    std::for_each(
      joint_positions.cbegin(), joint_positions.cend(),
      [&](const auto & j_pos) {
        jtc_point.positions.push_back(j_pos.second.at(i));
      });

    jt_msg.points.emplace_back(jtc_point);
  }

  std::for_each(
    joint_positions.cbegin(), joint_positions.cend(),
    [&](const auto & j_pos) {
      jt_msg.joint_names.push_back(j_pos.first);
    });

  jt_msg.header.stamp = now();
  return jt_msg;
}

ControllerTrajectories PlayMotion2::generate_controller_trajectories(
  const std::string & motion_key,
  const double extra_time) const
{
  ControllerTrajectories ct;
  for (const auto & controller : motion_controller_states_) {
    const auto trajectory = create_trajectory(controller, motion_key, extra_time);
    if (!trajectory.joint_names.empty()) {
      ct[controller.name] = trajectory;
    }
  }
  return ct;
}

FollowJTGoalHandleFutureResult PlayMotion2::send_trajectory(
  const std::string & controller_name,
  const JTMsg & trajectory)
{
  rclcpp_action::Client<FollowJT>::SharedPtr action_client;
  if (action_clients_.count(controller_name) != 0) {
    action_client = action_clients_[controller_name];
  } else {
    action_client = rclcpp_action::create_client<FollowJT>(
      this,
      "/" + controller_name + "/follow_joint_trajectory",
      client_cb_group_);
    action_clients_[controller_name] = action_client;
  }

  if (!action_client->wait_for_action_server(1s)) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "/" << controller_name <<
        "/follow_joint_trajectory action server not available after waiting");
    return {};
  }

  /// @todo fill rest of the fields ??
  auto goal = FollowJT::Goal();
  goal.trajectory = trajectory;

  auto goal_handle = action_client->async_send_goal(goal);

  if (!goal_handle.valid()) {
    return {};
  }
  std::future_status status;
  auto start_t = now();
  do {
    status = goal_handle.wait_for(0.1s);
    if (now() - start_t > kTimeout) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
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

PlayMotion2Action::Result::SharedPtr PlayMotion2::wait_for_results(
  const std::shared_ptr<GoalHandlePM2> goal_handle,
  std::list<FollowJTGoalHandleFutureResult> & futures_list,
  const double motion_time)
{
  bool failed = false;
  auto result = std::make_shared<PlayMotion2Action::Result>();

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->error = "Motion canceled";
    goal_handle->canceled(result);
    return result;
  }

  // Spin all futures and remove them when succeeded.
  // If one fails, set failed to true and returns false
  const auto successful_jt = [&](FollowJTGoalHandleFutureResult & future) {
      std::future_status status = future.wait_for(0.1s);
      if (status == std::future_status::ready) {
        if (future.get().code == rclcpp_action::ResultCode::SUCCEEDED) {
          return true;
        } else {
          failed = true;
          result->error = "Joint Trajectory failed";
          RCLCPP_ERROR_STREAM(get_logger(), result->error);
        }
      }
      return false;
    };

  // finish if failed, motions finished or timeout
  const double TIMEOUT = motion_time * 2.0 + 1.0;
  const rclcpp::Time init_time = now();
  bool on_time = true;
  do {
    futures_list.erase(
      std::remove_if(futures_list.begin(), futures_list.end(), successful_jt),
      futures_list.end());
    on_time = (now() - init_time).seconds() < TIMEOUT;

    auto current_states = filter_controller_states(
      get_controller_states(), "active", "joint_trajectory_controller/JointTrajectoryController");

    if (current_states != motion_controller_states_) {
      std::string controller_name = "";

      for (const auto & motion_controller : motion_controller_states_) {
        const auto controller = std::find_if(
          current_states.cbegin(), current_states.cend(),
          [&](const auto & current_controller_state) {
            return current_controller_state.name == motion_controller.name;
          });

        if (controller == current_states.end()) {
          controller_name = motion_controller.name;
        }
      }

      failed = true;
      result->error = "State of controller '" + controller_name +
        "' has changed while executing the motion";
      RCLCPP_ERROR_STREAM(get_logger(), result->error);
    }

    if (goal_handle->is_canceling()) {
      // cancel all sent goals
      for (const auto & client : action_clients_) {
        client.second->async_cancel_all_goals();
      }
      futures_list.clear();
      result->success = false;
      result->error = "Motion canceled";
      goal_handle->canceled(result);
    }
  } while (!failed && !futures_list.empty() && on_time);

  if (!on_time) {
    result->error = "Timeout exceeded while waiting for results";
    RCLCPP_ERROR_STREAM(get_logger(), result->error);
  }

  result->success = on_time && !failed;
  return result;
}

}  // namespace play_motion2
