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

from play_motion2_msgs.action import PlayMotion2
from play_motion2_msgs.msg import Motion
from play_motion2_msgs.srv import (
    AddMotion, GetMotionInfo, IsMotionReady, ListMotions, RemoveMotion
)

from rclpy import spin_until_future_complete
from rclpy.action import ActionClient
from rclpy.node import Node


class PlayMotion2ClientPy(Node):
    """A client for the Play Motion 2 service, allowing execution and management of motions."""

    last_succeeded: bool = False
    """Indicates whether the last motion execution was successful."""

    def __init__(self, node_name: str = 'play_motion2_client_py'):
        super().__init__(node_name)

        self.ac = ActionClient(self, PlayMotion2, 'play_motion2')

        self.list_motions_client = self.create_client(ListMotions, 'play_motion2/list_motions')
        self.is_motion_ready_client = self.create_client(
            IsMotionReady, 'play_motion2/is_motion_ready')
        self.get_motion_info_client = self.create_client(
            GetMotionInfo, 'play_motion2/get_motion_info')
        self.add_motion_client = self.create_client(AddMotion, 'play_motion2/add_motion')
        self.remove_motion_client = self.create_client(RemoveMotion, 'play_motion2/remove_motion')

        self.result_future = None
        self.goal_handle = None

        self.last_succeeded = False

    def run_motion(self, motion_name: str, skip_planning: bool = False,
                   motion_timeout: float = 120.0) -> bool:
        """
        Run a motion with the specified name.

        :param motion_name: Name of the motion to run.
        :param skip_planning: If True, skip the planning phase.
        :param motion_timeout: Timeout for the motion execution in seconds.
        :return: True if the motion was executed successfully, False otherwise.
        """
        if self.run_motion_async(motion_name, skip_planning):
            spin_until_future_complete(self, self.result_future, timeout_sec=motion_timeout)

            if self.result_future.done():
                result = self.result_future.result()
                if result and result.result.success:
                    self.get_logger().info(f'Motion {motion_name} executed successfully.')
                    return True
                else:
                    self.get_logger().error(
                        f'Motion {motion_name} failed to execute: {result.result.error}.')

        # The motion wasn't send successfully
        return False

    def run_motion_async(self, motion_name: str, skip_planning: bool = False) -> bool:
        """
        Run a motion asynchronously with the specified name.

        :param motion_name: Name of the motion to run.
        :param skip_planning: If True, skip the planning phase.
        :return: True if the motion was started successfully, False otherwise.
        """
        goal = PlayMotion2.Goal()
        goal.motion_name = motion_name
        goal.skip_planning = skip_planning

        if not self.ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False
        else:
            goal_future = self.ac.send_goal_async(goal)
            spin_until_future_complete(self, goal_future, timeout_sec=5.0)

            if goal_future.done():
                self.goal_handle = goal_future.result()

                if self.goal_handle is not None:
                    self.result_future = self.goal_handle.get_result_async()

                    return True

        # Clear internal state
        self.result_future = None
        self.goal_handle = None
        return False

    def wait_for_motion_result(self, timeout_sec: float = 5.0) -> bool:
        """
        Wait for the result of the last executed motion.

        :param timeout_sec: Timeout in seconds to wait for the result.
        :return: True if the result was received, False if it timed out.
        """
        if self.result_future is None:
            self.get_logger().error('No motion is currently running')
            return False

        spin_until_future_complete(self, self.result_future, timeout_sec=timeout_sec)

        if self.result_future.done():
            result = self.result_future.result()
            self.last_succeeded = result is not None and result.result.success

            self.result_future = None
            self.goal_handle = None

            return True
        else:
            self.get_logger().error('Motion result timed out')
            return False

    def is_running_motion(self) -> bool:
        """
        Check if a motion is currently running.

        :return: True if a motion is running, False otherwise.
        """
        return self.goal_handle is not None

    def list_motions(self) -> list[str]:
        """
        List all available motions.

        :return: A list of motion names.
        """
        if not self.list_motions_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('List motions service not available')
            return []
        request = ListMotions.Request()
        future = self.list_motions_client.call_async(request)
        spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            if future.result() is not None:
                return future.result().motion_keys
            else:
                self.get_logger().error('Failed to call list motions service')
        else:
            self.get_logger().error('List motions service call timed out')
        return []

    def is_motion_ready(self, motion_name: str) -> bool:
        """
        Check if a specific motion is ready to be executed.

        :param motion_name: Name of the motion to check.
        :return: True if the motion is ready, False otherwise.
        """
        if not self.is_motion_ready_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Is motion ready service not available')
            return False
        request = IsMotionReady.Request()
        request.motion_key = motion_name
        future = self.is_motion_ready_client.call_async(request)
        spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            if future.result() is not None:
                return future.result().is_ready
            else:
                self.get_logger().error('Failed to call is motion ready service')
        else:
            self.get_logger().error('Is motion ready service call timed out')
        return False

    def get_motion_info(self, motion_name: str) -> Motion:
        """
        Get information about a specific motion.

        :param motion_name: Name of the motion to get information about.
        :return: A dictionary containing motion information.
        """
        if not self.get_motion_info_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Get motion info service not available')
            return None
        request = GetMotionInfo.Request()
        request.motion_key = motion_name
        future = self.get_motion_info_client.call_async(request)
        spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            if future.result() is not None:
                return future.result().motion
            else:
                self.get_logger().error('Failed to call get motion info service')
        else:
            self.get_logger().error('Get motion info service call timed out')
        return None

    def add_motion(self, motion: Motion, overwrite: bool = False) -> bool:
        """
        Add a new motion to the system.

        :param motion: The motion to add.
        :param overwrite: If True, overwrite an existing motion with the same name.
        :return: True if the motion was added successfully, False otherwise.
        """
        if not self.add_motion_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Add motion service not available')
            return False
        request = AddMotion.Request()
        request.motion = motion
        request.overwrite = overwrite
        future = self.add_motion_client.call_async(request)
        spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            if future.result() is not None:
                return future.result().success
            else:
                self.get_logger().error('Failed to call add motion service')
        else:
            self.get_logger().error('Add motion service call timed out')
        return False

    def remove_motion(self, motion_name: str) -> bool:
        """
        Remove a motion from the system.

        :param motion_name: Name of the motion to remove.
        :return: True if the motion was removed successfully, False otherwise.
        """
        if not self.remove_motion_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Remove motion service not available')
            return False
        request = RemoveMotion.Request()
        request.motion_key = motion_name
        future = self.remove_motion_client.call_async(request)
        spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            if future.result() is not None:
                return future.result().success
            else:
                self.get_logger().error('Failed to call remove motion service')
        else:
            self.get_logger().error('Remove motion service call timed out')
        return False
