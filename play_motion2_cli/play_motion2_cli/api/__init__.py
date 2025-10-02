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

from play_motion2 import PlayMotion2ClientPy


class MotionNameCompleter:
    """Callable returning a list of motion names."""

    def __call__(self, prefix, parsed_args, **kwargs):
        try:
            rclpy.init()
            play_motion2_client = PlayMotion2ClientPy('cli_play_motion2_client_py_completer')
            list_motions = play_motion2_client.list_motions()
            rclpy.try_shutdown()
            return list_motions
        except Exception:
            return []
