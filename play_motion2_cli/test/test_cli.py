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

import contextlib
import os
import sys
import unittest

from fixtures.play_motion_server import EXPECTED_MOTIONS, MOTIONS_INFO

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rclpy.utilities import get_available_rmw_implementations


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True)


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_play_motion_server_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'play_motion_server.py'
    )
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}

    run_zenoh = (rmw_implementation == 'rmw_zenoh_cpp')

    on_exit_actions = []
    if run_zenoh:
        on_exit_actions.extend([
            ExecuteProcess(
                cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
                name='rmw-zenohd',
                additional_env=additional_env,
                output='screen',
            )
        ])

    on_exit_actions.extend([
        Node(
            executable=sys.executable,
            arguments=[path_to_play_motion_server_script],
            name='play_motion_server',
            namespace='play_motion2',
            additional_env=additional_env,
        ),
        launch_testing.actions.ReadyToTest()
    ])

    return LaunchDescription([
        # Always restart daemon to isolate tests.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=on_exit_actions,
                    additional_env=additional_env,
                )
            ]
        ),
    ])


class TestROS2PLayMotionCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_service_command(self, arguments):
            service_command_action = ExecuteProcess(
                cmd=['ros2', 'play_motion', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2play_motion-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, service_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_prefixes=[
                        'waiting for service to become available...',
                        '/launch_ros'  # cope with launch_ros internal node.
                    ],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as service_command:
                yield service_command
        cls.launch_service_command = launch_service_command

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_service(self):
        with self.launch_service_command(arguments=['list']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=sorted(EXPECTED_MOTIONS.keys()),
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_motion_ready(self):
        with self.launch_service_command(arguments=['list', '--is-ready']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        expected_lines = [
            f"{motion} [{'Ready' if EXPECTED_MOTIONS[motion] else 'Not ready'}]"
            for motion in sorted(EXPECTED_MOTIONS.keys())
        ]
        assert launch_testing.tools.expect_output(
            expected_lines=expected_lines,
            text=service_command.output.strip(),
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_get_info_service(self):
        with self.launch_service_command(arguments=['info', 'nod']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        data = MOTIONS_INFO['nod']
        assert launch_testing.tools.expect_output(
            expected_lines=[
                f"Key: {data['key']}",
                f"Description: {data['description']}",
                '',
                'Joints:'
            ] + [f'  {joint}' for joint in data['joints']],
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_get_info_service_simple_verbose(self):
        with self.launch_service_command(arguments=['info', 'wave', '-v']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        data = MOTIONS_INFO['wave']
        assert launch_testing.tools.expect_output(
            expected_lines=[
                f"Key: {data['key']}",
                f"Name: {data['name']}",
                f"Usage: {data['usage']}",
                f"Description: {data['description']}",
                '',
                'Joints and positions:',
                'shoulder_joint   0.0000',
                'elbow_joint      1.2000 ',
                'Times from start:',
            ] + [
                f'  {t:.2f} s' for t in data['times_from_start']
            ],
            text=service_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_get_info_service_verbose(self):
        with self.launch_service_command(arguments=['info', 'nod', '-v']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        data = MOTIONS_INFO['nod']
        assert launch_testing.tools.expect_output(
            expected_lines=[
                f"Key: {data['key']}",
                f"Name: {data['name']}",
                f"Usage: {data['usage']}",
                f"Description: {data['description']}",
                '',
                'Joints and positions:',
                'Joint         pos1     pos2     pos3',
                'neck_pitch   0.2000  -0.2000   0.2000',
                'Times from start:',
            ] + [f'  {t:.2f} s' for t in data['times_from_start']],
            text=service_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_get_info_service_error(self):
        with self.launch_service_command(arguments=['info', 'unknown_motion']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Unknown motion 'unknown_motion'",
            ],
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_playmotion2_action_not_exist(self):
        with self.launch_service_command(arguments=['run', 'unknown_motion']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Motion 'unknown_motion' is not ready or does not exist",
            ],
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_playmotion2_action_not_ready(self):
        with self.launch_service_command(arguments=['run', 'nod']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Motion 'nod' is not ready or does not exist",
            ],
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_playmotion2_action(self):
        with self.launch_service_command(arguments=['run', 'wave']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Executing motion 'wave'... (press Ctrl-C to cancel)",
                'The motion has been executed correctly',
            ],
            text=service_command.output,
            strict=True
        )
