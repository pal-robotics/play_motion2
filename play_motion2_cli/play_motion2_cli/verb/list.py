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

from play_motion2_cli.api import create_playmotion_client
from play_motion2_cli.verb import VerbExtension
from ros2cli.node.strategy import add_arguments


class ListVerb(VerbExtension):
    """Output a list of play_motion keys."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '-r', '--is-ready', action='store_true',
            help='Additionally show if the motion is ready')

    def main(self, *, args):
        with create_playmotion_client('cli_play_motion2_client_py') as play_motion2_client:
            play_motion_list = sorted(play_motion2_client.list_motions())

            for name in play_motion_list:
                if args.is_ready:
                    motion = play_motion2_client.is_motion_ready(name)
                    print(f"{name} [{'Ready' if motion else 'Not ready'}]")
                else:
                    print(name)
