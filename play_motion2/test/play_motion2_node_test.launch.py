# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

import os
import pathlib
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing

TEST_DIR = str(pathlib.Path(__file__).resolve().parent)


def generate_test_description():

    rrbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            TEST_DIR + '/rrbot/rrbot.launch.py'
        )
    )

    play_motion2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('play_motion2'), 'launch', 'play_motion2.launch.py'])),
        launch_arguments={
            'motions_file': TEST_DIR + '/play_motion2_config.yaml',
            'use_sim_time': 'False',
        }.items()
    )

    play_motion2_node_test = launch_testing.actions.GTest(
        path=os.path.join(get_package_prefix('play_motion2'), 'lib',
                          'play_motion2', 'play_motion2_node_test'),
        output='screen')

    ld = LaunchDescription()

    ld.add_action(rrbot)
    ld.add_action(play_motion2)
    ld.add_action(play_motion2_node_test)

    ld.add_action(launch_testing.actions.ReadyToTest())

    context = {'play_motion2_node_test': play_motion2_node_test}

    return ld, context


class TestPlayMotion(unittest.TestCase):

    def test_wait(self, play_motion2_node_test, proc_info):
        proc_info.assertWaitForShutdown(process=play_motion2_node_test, timeout=(60))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, play_motion2_node_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=play_motion2_node_test)
