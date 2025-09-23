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
from play_motion2_cli.api import list_motions, is_motion_ready
from play_motion2_cli.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of play_motion keys."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '-r', '--motion-ready', action='store_true',
            help='Additionally show if the motion is ready')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            play_motion_list = list_motions(node=node)

            for name in play_motion_list:
                if args.motion_ready:
                    motion = is_motion_ready(node, name)
                    print(f"{name} [{'Ready' if motion.is_ready else 'Not ready'}]")
                else:
                    print(name)
