# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from play_motion2_msgs.srv import IsMotionReady, GetMotionInfo, ListMotions
from play_motion2_msgs.action import PlayMotion2

from rclpy.action import ActionClient
from rclpy.task import Future
from ros2cli.node.strategy import NodeStrategy


def call_service(node, srv_module, service_name, request):
    if not check_service_exist(node, f'/{service_name}'):
        raise RuntimeError(f"Service '/{service_name}' not available.")

    cli = node.create_client(srv_module, service_name)
    if not cli.wait_for_service(5.0):
        raise RuntimeError(f"Service '/{service_name}' not available.")

    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        return future.result()
    else:
        raise RuntimeError(f'Exception while calling service: {future.exception()}')


def check_service_exist(node, service):
    service_names_and_types = node.get_service_names_and_types()
    return service in dict(service_names_and_types)


def list_motions(node):
    play_motion_list = call_service(
        node, ListMotions, 'play_motion2/list_motions', ListMotions.Request())
    return sorted(play_motion_list.motion_keys)


def is_motion_ready(node, motion):
    request = IsMotionReady.Request()
    request.motion_key = motion
    return call_service(node, IsMotionReady, 'play_motion2/is_motion_ready', request)


def get_motion_info(node, motion):
    request = GetMotionInfo.Request()
    request.motion_key = motion
    service_call = call_service(node, GetMotionInfo, 'play_motion2/get_motion_info', request)

    if not service_call.motion.key:
        raise RuntimeError(f"Unknown motion '{motion}'")
    return service_call.motion


def call_action(node, action_module, action_name, goal):
    if not check_action_exist(node, action_name):
        raise RuntimeError(f"Action '{action_name}' not available.")

    action_client = ActionClient(node, action_module, action_name)
    if not action_client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError(f"Action '{action_name}' not available.")

    done_future = Future()
    goal_handle = None

    def goal_response_cb(future):
        nonlocal goal_handle
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"Motion '{goal.motion_name}' rejected")
            done_future.set_result(None)
            return
        print(f"Running {goal.motion_name} motion")
        goal_handle.get_result_async().add_done_callback(
            lambda r: done_future.set_result(r.result().result)
        )

    action_client.send_goal_async(goal).add_done_callback(goal_response_cb)

    try:
        while rclpy.ok() and not done_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if goal_handle:
            goal_handle.cancel_goal_async()
        raise

    return done_future.result()


def check_action_exist(node, action):
    action_names_and_types = node.get_action_names_and_types()
    return action in dict(action_names_and_types)


def run_play_motion(node, motion, skip_planning):
    goal = PlayMotion2.Goal()
    goal.motion_name = motion
    goal.skip_planning = skip_planning
    action_result = call_action(node, PlayMotion2, '/play_motion2', goal)

    if not action_result:
        raise RuntimeError(f"Motion '{motion}' does not exist")
    return action_result


class MotionNameCompleter:
    """Callable returning a list of motion names."""

    def __call__(self, prefix, parsed_args, **kwargs):
        try:
            with NodeStrategy(parsed_args) as node:
                return list_motions(node)
        except Exception:
            return []
