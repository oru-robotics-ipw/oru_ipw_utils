#!/usr/bin/env python2

from __future__ import absolute_import, print_function

import argparse

import attr
import numpy as np
import rosbag
import rospy
from pathlib2 import Path


@attr.s
class AccelerationEntry(object):
    """Data from log"""
    ts = attr.ib(type=rospy.Time)
    cmd_vel = attr.ib(type=np.ndarray)  # Linear, Angular
    encoder_ticks = attr.ib(type=np.ndarray, default=None)  # Left, right
    encoder_deltas = attr.ib(type=np.ndarray, default=None)  # Left, right
    location = attr.ib(type=np.ndarray, default=None)  # x, y
    theta = attr.ib(type=float, default=None)

    def get_speed(self, prev):
        """Speed since the previous entry

        :type prev: AccelerationEntry
        :rtype: float
        """
        delta = np.linalg.norm(self.location - prev.location)
        t_delta = (self.ts - prev.ts).to_sec()
        if t_delta == 0:
            return 0
        else:
            return delta / t_delta

    def get_wheel_speeds(self, prev):
        """Speed since the previous entry

        :type prev: AccelerationEntry
        :rtype: np.ndarray
        """
        t_delta = (self.ts - prev.ts).to_sec()
        if t_delta == 0:
            return np.array([0, 0])
        else:
            return self.encoder_deltas / t_delta


def main():
    """Main program entrypoint"""
    parser = argparse.ArgumentParser(description='Analyse speed and acceleration in a bag file')
    parser.add_argument('bag_file',
                        type=Path,
                        help='Path to bag file')
    parser.add_argument('analysis',
                        choices=['odom', 'encoder'],
                        help="Type of analysis to perform")
    args = parser.parse_args()

    analysis = args.analysis  # type: str

    last_cmd_vel = np.array([0, 0])
    entries = []
    ts_to_entry = {}

    topics_to_collect = ['/hrp/cmd_vel']

    if analysis == 'odom':
        topics_to_collect.append('/hrp/odom')
    elif analysis == 'encoder':
        topics_to_collect.append('/hrp/wheel_encoder')

    with rosbag.Bag(str(args.bag_file)) as bag:
        for topic, msg, t in bag.read_messages(topics=topics_to_collect):
            if hasattr(msg, 'header'):
                ts = msg.header.stamp  # type: rospy.Time
            else:
                ts = None
            if topic == '/hrp/cmd_vel':
                last_cmd_vel = np.array([msg.linear.x, msg.angular.z])
                continue

            ns = ts.to_nsec()
            if ns in ts_to_entry:
                entry = ts_to_entry[ns]
            else:
                entry = AccelerationEntry(ts, last_cmd_vel)
                entries.append(entry)
                ts_to_entry[ns] = entry

            if topic == '/hrp/odom':
                entry.location = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
                entry.theta = np.arcsin(msg.pose.pose.orientation.z / 2.0)
            elif topic == '/hrp/wheel_encoder':
                entry.encoder_deltas = np.array([msg.lwheel, msg.rwheel])
                entry.encoder_ticks = np.array([msg.lwheelAccum, msg.rwheelAccum])
            else:
                raise Exception('Unexpected message type: %r' % type(msg))

    # Now print the data
    if analysis == 'odom':
        first_ts = entries[0].ts
        print('Time,LCmd,ACmd,X,Y,Theta,Speed')
        prev_entry = entries[0]
        for entry in entries:
            print('%r,%r,%r,%r,%r,%r,%r' % ((entry.ts - first_ts).to_sec(),
                                            entry.cmd_vel[0],
                                            entry.cmd_vel[1],
                                            entry.location[0],
                                            entry.location[1],
                                            entry.theta,
                                            entry.get_speed(prev_entry)))
            prev_entry = entry
    elif analysis == 'encoder':
        first_ts = entries[0].ts
        print('Time,LCmd,ACmd,EncL,EncR')
        prev_entry = entries[0]
        for entry in entries:
            wheel_speeds = entry.get_wheel_speeds(prev_entry)
            print('%r,%r,%r,%r,%r' % ((entry.ts - first_ts).to_sec(),
                                      entry.cmd_vel[0],
                                      entry.cmd_vel[1],
                                      wheel_speeds[0],
                                      wheel_speeds[1]))
            prev_entry = entry
