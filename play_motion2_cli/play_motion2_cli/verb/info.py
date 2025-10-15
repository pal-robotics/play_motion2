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

from play_motion2_cli.api import add_motion_name_argument, create_playmotion_client
from play_motion2_cli.verb import VerbExtension
from ros2cli.node.strategy import add_arguments


class InfoVerb(VerbExtension):
    """Display information about a motion."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        add_motion_name_argument(parser)
        parser.add_argument(
            '--verbose',
            '-v',
            action='store_true',
            help='Prints detailed information like the motion name, usage, joint positions, '
                 'and Times from start')

    def main(self, *, args):
        with create_playmotion_client('cli_play_motion2_client_py') as play_motion2_client:

            motion = play_motion2_client.get_motion_info(args.motion_name)
            if motion is None:
                return
            if not motion.key:
                print(f"Unknown motion '{args.motion_name}'")
                return

            print(f'Key: {motion.key}')
            if args.verbose:
                print(f'Name: {motion.name}')
                print(f'Usage: {motion.usage}')
            print(f'Description: {motion.description}')

            if args.verbose:
                max_joint_len = max(len(joint) for joint in motion.joints)
                n_positions = len(motion.times_from_start)
                header = f"{'Joint':<{max_joint_len + 3}} " + '     '.join(
                    f'pos{i+1}' for i in range(n_positions))
                separation = '-' * (len(header) + 1)

                print('\nJoints and positions:')
                print(separation)
                if n_positions > 1:
                    print(header)
                    print(separation)

                for j, joint in enumerate(motion.joints):
                    row = f'{joint:<{max_joint_len}} '
                    for i in range(n_positions):
                        pos = motion.positions[i * len(motion.joints) + j]
                        row += f'{pos:8.4f} '
                    print(row)
            else:
                print('\nJoints:')
                for joint in motion.joints:
                    print(f'  {joint}')

            if args.verbose:
                print('\nTimes from start:')
                for t in motion.times_from_start:
                    print(f'  {t:.2f} s')
