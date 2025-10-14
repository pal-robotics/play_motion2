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

import sys

from play_motion2_msgs.action import PlayMotion2
from play_motion2_msgs.msg import Motion
from play_motion2_msgs.srv import GetMotionInfo, IsMotionReady, ListMotions

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

EXPECTED_MOTIONS = {
    'nod': False,
    'wave': True,
    'bow': False
}

MOTIONS_INFO = {
    'nod': {
        'key': 'nod',
        'name': 'Nod Head',
        'usage': 'Affirmative gesture',
        'description': "Moves head up and down like saying 'yes'",

        'joints': ['neck_pitch'],
        'positions': [0.2, -0.2, 0.2],
        'times_from_start': [0.0, 1.0, 2.0]
    },
    'wave': {
        'key': 'wave',
        'name': 'Wave Hand',
        'usage': 'Greeting',
        'description': 'Waves hand left and right',

        'joints': ['shoulder_joint', 'elbow_joint'],
        'positions': [0.0, 1.2],
        'times_from_start': [0.2]
    }
}


class DummyPlayMotionServer(Node):

    def __init__(self):
        super().__init__('play_motion2')
        self.list_server = self.create_service(ListMotions, 'list_motions', self.list_callback)
        self.is_motion_ready_server = self.create_service(
            IsMotionReady, 'is_motion_ready', self.is_motion_ready_callback)
        self.get_info_server = self.create_service(
            GetMotionInfo, 'get_motion_info', self.get_info_callback)
        self.action_server = ActionServer(
            self,
            PlayMotion2,
            '/play_motion2',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )

    def list_callback(self, request, response):
        response.motion_keys = list(EXPECTED_MOTIONS.keys())
        return response

    def is_motion_ready_callback(self, request, response):
        response.is_ready = EXPECTED_MOTIONS.get(request.motion_key, False)
        return response

    def get_info_callback(self, request, response):
        data = MOTIONS_INFO.get(request.motion_key)
        if data is None:
            return response

        motion_info = Motion()
        motion_info.key = data['key']
        motion_info.name = data['name']
        motion_info.usage = data['usage']
        motion_info.description = data['description']
        motion_info.joints = data['joints']
        motion_info.positions = data['positions']
        motion_info.times_from_start = data['times_from_start']
        response.motion = motion_info
        return response

    def goal_callback(self, goal_request):
        if goal_request.motion_name in EXPECTED_MOTIONS:
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    async def execute_callback(self, goal_handle):
        result = PlayMotion2.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    node = DummyPlayMotionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
