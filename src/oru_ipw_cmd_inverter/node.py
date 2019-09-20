"""ROS node that inverts the linear component of cmd_vel"""

from __future__ import (print_function, absolute_import, division)

import rospy
from geometry_msgs.msg import Twist


class Node(object):
    """Main node class"""

    def __init__(self):
        """Constructor"""
        self._sub = rospy.Subscriber('cmd_vel_inverted',
                                     Twist,
                                     callback=self._callback_cmd_vel,
                                     queue_size=1)
        self._cmd_vel_pub = rospy.Publisher('cmd_vel',
                                            Twist,
                                            latch=False,
                                            queue_size=2)

    def _callback_cmd_vel(self, msg):
        """Callback from cmd_vel

        :type msg: Twist
        :return:
        """
        msg.linear.x *= -1
        self._cmd_vel_pub.publish(msg)


def main():
    """Main program entry point"""
    rospy.init_node("oru_ipw_cmd_inverter")

    ros_node = Node()

    # Spin on ROS message bus
    rospy.spin()
