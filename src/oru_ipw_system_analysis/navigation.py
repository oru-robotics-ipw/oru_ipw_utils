from __future__ import absolute_import, print_function

import json

import rospy
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion

from .analysis import Analysis

# Create a mapping from values to string constants
_GOAL_STATUS_RESOLVER = {}
for attr in dir(GoalStatus):
    value = getattr(GoalStatus, attr)
    if attr.startswith('_'):
        continue
    if isinstance(value, int):
        _GOAL_STATUS_RESOLVER[value] = attr


class NavigationAnalysis(Analysis):
    """Navigation analysis"""

    def analyse(self):
        events = []
        for bag in self._bags:
            for topic, msg, t in bag.read_messages(topics=('/move_base/goal', '/move_base/result')):
                ts = msg.header.stamp  # type: rospy.Time
                if topic == '/move_base/result':
                    # assert isinstance(msg, MoveBaseActionResult)
                    events.append({'action': 'nav_end',
                                   'ts': ts.to_sec(),
                                   'goal_id': msg.status.goal_id.id,
                                   'status': _GOAL_STATUS_RESOLVER[msg.status.status]})
                elif topic == '/move_base/goal':
                    # assert isinstance(msg, MoveBaseActionGoal)
                    orientation = msg.goal.target_pose.pose.orientation
                    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
                    events.append({'action': 'nav_start',
                                   'ts': msg.goal.target_pose.header.stamp.to_sec(),
                                   'goal_id': msg.goal_id.id,
                                   'x': msg.goal.target_pose.pose.position.x,
                                   'y': msg.goal.target_pose.pose.position.y,
                                   'theta': yaw})
        print(json.dumps(events, indent=2))
