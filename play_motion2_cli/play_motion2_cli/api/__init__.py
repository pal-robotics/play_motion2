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

from argparse import ArgumentParser
from contextlib import contextmanager

from play_motion2 import PlayMotion2ClientPy
import rclpy


class MotionNameCompleter:
    """Callable returning a list of motion names."""

    def __call__(self, prefix, parsed_args, **kwargs):
        try:
            with create_playmotion_client('cli_play_motion2_completer') as play_motion2_client:
                return play_motion2_client.list_motions()
        except Exception:
            return []


def add_motion_name_argument(verb_subparser: ArgumentParser):
    arg = verb_subparser.add_argument(
        'motion_name',
        help="Name of the motion (e.g. 'head_down')"
    )
    arg.completer = MotionNameCompleter()


@contextmanager
def create_playmotion_client(name):
    try:
        if not rclpy.ok():
            rclpy.init()
        node = PlayMotion2ClientPy(name)
        yield node
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
