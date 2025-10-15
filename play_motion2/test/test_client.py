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

from threading import Thread
import time
import unittest

from play_motion2 import PlayMotion2ClientPy
from play_motion2_msgs.action import PlayMotion2
from play_motion2_msgs.msg import Motion
from play_motion2_msgs.srv import (
    AddMotion, GetMotionInfo, IsMotionReady, ListMotions, RemoveMotion
)

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class ClientTest(unittest.TestCase):
    _runner_srv = Thread()
    _runner_cli = Thread()

    motion_list = [
        Motion(key='motion1', description='Test motion 1'),
        Motion(key='motion2', description='Test motion 2'),
        Motion(key='motion3', description='Test motion 3'),
    ]

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._executor_srv = MultiThreadedExecutor()
        self._executor_cli = SingleThreadedExecutor()

        self._node_srv = rclpy.create_node('play_motion2_server_test')
        self._ac_callbackgroup = MutuallyExclusiveCallbackGroup()

        # Create playmotion AS
        self._playmotion_action_server = ActionServer(
            self._node_srv, PlayMotion2, 'play_motion2', self.execute_callback,
            callback_group=self._ac_callbackgroup)

        # Create service servers
        self._list_motions_srv = self._node_srv.create_service(
            ListMotions, 'play_motion2/list_motions', self.list_motions_callback)
        self._is_motion_ready_srv = self._node_srv.create_service(
            IsMotionReady, 'play_motion2/is_motion_ready', self.is_motion_ready_callback)
        self._get_motion_info_srv = self._node_srv.create_service(
            GetMotionInfo, 'play_motion2/get_motion_info', self.get_motion_info_callback)
        self._add_motion_srv = self._node_srv.create_service(
            AddMotion, 'play_motion2/add_motion', self.add_motion_callback)
        self._remove_motion_srv = self._node_srv.create_service(
            RemoveMotion, 'play_motion2/remove_motion', self.remove_motion_callback)

        self._executor_srv.add_node(self._node_srv)
        self._runner_srv = Thread(target=lambda exec: exec.spin(),
                                  args=([self._executor_srv]), daemon=True).start()

        self.cli = PlayMotion2ClientPy()
        self._executor_cli.add_node(self.cli)
        self._runner_cli = Thread(target=lambda exec: exec.spin(),
                                  args=([self._executor_cli]), daemon=True).start()

    def tearDown(self):
        self._executor_srv.shutdown()
        self._executor_srv.remove_node(self._node_srv)
        self._node_srv.destroy_node()

        self._executor_cli.shutdown()
        self._executor_cli.remove_node(self.cli)
        self.cli.destroy_node()

    def list_motions_callback(self, _, response: ListMotions.Response):
        response.motion_keys = [motion.key for motion in self.motion_list]
        return response

    def is_motion_ready_callback(self, request: IsMotionReady.Request,
                                 response: IsMotionReady.Response):
        response.is_ready = any(motion.key == request.motion_key for motion in self.motion_list)
        return response

    def get_motion_info_callback(self, request: GetMotionInfo.Request,
                                 response: GetMotionInfo.Response):
        for motion in self.motion_list:
            if motion.key == request.motion_key:
                response.motion = motion
                return response

        response.motion = Motion()
        return response

    def add_motion_callback(self, request: AddMotion.Request,
                            response: AddMotion.Response):
        if request.motion.key in [motion.key for motion in self.motion_list]:
            if request.overwrite:
                # Update existing motion
                for i, motion in enumerate(self.motion_list):
                    if motion.key == request.motion.key:
                        self.motion_list[i] = request.motion
                        response.success = True
            else:
                response.success = False
        else:
            self.motion_list.append(request.motion)
            response.success = True

        return response

    def remove_motion_callback(self, request: RemoveMotion.Request,
                               response: RemoveMotion.Response):
        for motion in self.motion_list:
            if motion.key == request.motion_key:
                self.motion_list.remove(motion)
                response.success = True
                return response

        response.success = False
        return response

    def execute_callback(self, goal_handle):
        if any(motion.key == goal_handle.request.motion_name
               for motion in self.motion_list) is False:
            goal_handle.abort()
            result = PlayMotion2.Result()
            result.success = False
            result.error = f'Motion {goal_handle.request.motion_name} not found'
            return result

        self.current_motion = goal_handle.request.motion_name
        self.skip_planning = goal_handle.request.skip_planning

        self.is_done = False
        self.is_success = True
        self.error_message = ''
        while not self.is_done:
            time.sleep(0.01)

        result = PlayMotion2.Result()
        result.success = self.is_success
        result.error = self.error_message
        goal_handle.succeed()

        return result

    def finish_motion(self, success=True, error_message=''):
        self.is_done = True
        self.is_success = success
        self.error_message = error_message

    def test_list_motions(self):
        response_motion_list = self.cli.list_motions()
        self.assertIsNotNone(response_motion_list)
        self.assertEqual(len(response_motion_list), len(self.motion_list))
        for motion in self.motion_list:
            self.assertIn(motion.key, response_motion_list)

    def test_is_motion_ready(self):
        for motion in self.motion_list:
            is_ready = self.cli.is_motion_ready(motion.key)
            self.assertTrue(is_ready, f'Motion {motion.key} should be ready')

        is_ready = self.cli.is_motion_ready('non_existent_motion')
        self.assertFalse(is_ready, 'Non-existent motion should not be ready')

    def test_get_motion_info(self):
        for motion in self.motion_list:
            motion_info = self.cli.get_motion_info(motion.key)
            self.assertIsNotNone(motion_info)
            self.assertEqual(motion_info.key, motion.key)
            self.assertEqual(motion_info.description, motion.description)

        motion_info = self.cli.get_motion_info('non_existent_motion')
        self.assertIsNotNone(motion_info)
        self.assertEqual(motion_info.key, '', 'Non-existent motion should return empty key')

    def test_add_motion(self):
        new_motion = Motion(key='new_motion', description='New test motion')
        success = self.cli.add_motion(new_motion)
        self.assertTrue(success, 'Adding new motion should succeed')

        # Verify the motion was added
        motion_info = self.cli.get_motion_info(new_motion.key)
        self.assertIsNotNone(motion_info)
        self.assertEqual(motion_info.key, new_motion.key)
        self.assertEqual(motion_info.description, new_motion.description)

        # Attempt to update
        new_motion.description = 'Updated test motion'

        # Try to add the same motion without overwrite
        success = self.cli.add_motion(new_motion, overwrite=False)
        self.assertFalse(success, 'Adding existing motion without overwrite should fail')

        # Try to add the same motion with overwrite
        success = self.cli.add_motion(new_motion, overwrite=True)
        self.assertTrue(success, 'Adding existing motion with overwrite should succeed')
        # Verify the motion was added again
        motion_info = self.cli.get_motion_info(new_motion.key)
        self.assertIsNotNone(motion_info)
        self.assertEqual(motion_info.key, new_motion.key)
        self.assertEqual(motion_info.description, new_motion.description)

    def test_remove_motion(self):
        # Add a motion to remove
        motion_to_remove = Motion(key='motion_to_remove', description='Motion to be removed')
        self.cli.add_motion(motion_to_remove)

        # Verify the motion was added
        motion_info = self.cli.get_motion_info(motion_to_remove.key)
        self.assertIsNotNone(motion_info)

        # Remove the motion
        success = self.cli.remove_motion(motion_to_remove.key)
        self.assertTrue(success, 'Removing existing motion should succeed')

        # Verify the motion was removed
        motion_info = self.cli.get_motion_info(motion_to_remove.key)
        self.assertIsNotNone(motion_info)
        self.assertEqual(motion_info.key, '', 'Non-existent motion should return empty key')

    def test_play_motion(self):
        # Test playing a motion that exists
        motion_name = 'motion1'
        self.assertTrue(self.cli.run_motion_async(motion_name))

        time.sleep(0.1)  # Allow some time for the action to start

        # Check if the motion is running
        self.assertTrue(self.cli.is_running_motion(), f'Motion {motion_name} should be running')

        # Finish the motion
        self.finish_motion(success=True)
        self.assertTrue(self.cli.wait_for_motion_result(timeout_sec=0.5))

        self.assertTrue(self.cli.last_succeeded,
                        f'Motion {motion_name} should have succeeded')
        self.assertFalse(self.cli.is_running_motion(),
                         f'Motion {motion_name} should not be running after finish')

    def test_play_motion_non_existent(self):
        # Test playing a motion that does not exist
        motion_name = 'non_existent_motion'
        self.assertFalse(self.cli.run_motion(motion_name))
