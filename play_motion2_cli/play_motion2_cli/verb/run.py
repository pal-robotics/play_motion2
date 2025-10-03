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

from ros2cli.node.strategy import add_arguments

from play_motion2_cli.api import MotionNameCompleter
from play_motion2 import PlayMotion2ClientPy
from play_motion2_cli.verb import VerbExtension


class RunVerb(VerbExtension):
    """Execute a motion."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            'motion_name',
            help="Name of the motion to run (e.g. 'head_down')")
        parser.add_argument(
            '--skip-planning',
            action='store_true',
            help='Whether to skip planning for approaching to the first position or not.')
        arg.completer = MotionNameCompleter()

    def main(self, *, args):
        rclpy.init()
        play_motion2_client = PlayMotion2ClientPy('cli_play_motion2_client_py')

        if not play_motion2_client.is_motion_ready(args.motion_name):
            print(f"Motion '{args.motion_name}' is not ready or does not exist")
            return
        print(f"Executing motion '{args.motion_name}'... (press Ctrl-C to cancel)")

        play_motion2_client.run_motion_async(args.motion_name, args.skip_planning)

        try:
            while rclpy.ok() and not play_motion2_client.result_future.done():
                rclpy.spin_once(play_motion2_client, timeout_sec=0.1)
        except KeyboardInterrupt:
            if play_motion2_client.goal_handle:
                play_motion2_client.goal_handle.cancel_goal_async()
                return
        result_response = (play_motion2_client.last_succeeded if False else
                           play_motion2_client.result_future.result())

        if result_response.result.success:
            print('The motion has been executed correctly')

        play_motion2_client.destroy_node()
        rclpy.try_shutdown()
