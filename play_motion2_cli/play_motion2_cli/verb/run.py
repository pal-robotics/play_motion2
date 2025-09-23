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

from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy

from play_motion2_cli.api import MotionNameCompleter
from play_motion2_cli.api import run_play_motion
from play_motion2_cli.verb import VerbExtension


class RunVerb(VerbExtension):
    """Execute a motion."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            'motion_name',
            help="Name of the motion to run (e.g. 'head_down')")
        parser.add_argument(
            '--skip_planning',
            action='store_true',
            help='Whether to skip planning for approaching to the first position or not.')
        arg.completer = MotionNameCompleter()

    def main(self, *, args):
        with NodeStrategy(args) as node:
            result = run_play_motion(node, args.motion_name, args.skip_planning)
            if result.success:
                print('The motion has been executed correctly')
            else:
                print(result.error)
