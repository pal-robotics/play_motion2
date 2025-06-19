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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Specify whether to use simulation time or not. ')

    motions_config = DeclareLaunchArgument(
        'motions_file',
        description='Yaml file with the info of the motions. ')

    motion_planner_config = DeclareLaunchArgument(
        'motion_planner_config',
        default_value=os.path.join(
            get_package_share_directory('play_motion2'), 'config', 'motion_planner_config.yaml'),
        description='Configuration for the motion planner. ')

    play_motion2 = Node(
        package='play_motion2',
        executable='play_motion2_node',
        output='both',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('motions_file'),
            LaunchConfiguration('motion_planner_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/play_motion2_executor/robot_description', '/robot_description'),
            ('/play_motion2_executor/robot_description_semantic', '/robot_description_semantic'),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(sim_time_arg)
    ld.add_action(motions_config)
    ld.add_action(motion_planner_config)
    ld.add_action(play_motion2)

    return ld
