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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_pal.include_utils import include_launch_py_description
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Whether run rviz',
        choices=['True', 'False'])

    robot_description_path = os.path.join(
        get_package_share_directory('play_motion2'), 'test', 'rrbot.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    rrbot_controllers = os.path.join(
        get_package_share_directory('play_motion2'),
        'test', 'controller_manager.yaml')

    rviz_config_file = os.path.join(
        get_package_share_directory('play_motion2'), 'test', 'rrbot.rviz')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, rrbot_controllers],
        output='both')

    controllers = include_launch_py_description(
        'play_motion2', ['test', 'controllers.launch.py'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription()

    ld.add_action(use_rviz)

    ld.add_action(control_node)
    ld.add_action(controllers)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
