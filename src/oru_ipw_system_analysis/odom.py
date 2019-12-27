from __future__ import absolute_import, print_function

import numpy as np
import rospy

from .analysis import Analysis

_MANUAL_CONTROL = 0
_AUTO_CONTROL = 1


class OdomAnalysis(Analysis):
    """Odometry analysis"""

    def analyse(self):
        old_position = None
        old_ts = None
        ts_start = None
        distance_total = 0.0
        control_mode = _MANUAL_CONTROL
        distance_categories = {_MANUAL_CONTROL: 0.0, _AUTO_CONTROL: 0.0}
        max_speed = 0.0
        time_active = rospy.Duration(0)
        time_categories = {_MANUAL_CONTROL: rospy.Duration(0), _AUTO_CONTROL: rospy.Duration(0)}
        for bag in self._bags:
            for topic, msg, t in bag.read_messages(topics=('/hrp/odom', '/move_base/goal', '/move_base/result')):
                ts = msg.header.stamp  # type: rospy.Time
                if not ts_start:
                    ts_start = ts
                if topic == '/hrp/odom':
                    # assert isinstance(msg, Odometry)
                    new_position = msg.pose.pose.position
                    vec = np.array([new_position.x, new_position.y])
                    if old_position is not None:
                        delta_vec = vec - old_position  # type: np.ndarray
                        delta_distance = np.linalg.norm(delta_vec)  # type: float
                        # Compute running distance
                        distance_total += delta_distance
                        distance_categories[control_mode] += delta_distance
                        # Compute active time
                        if delta_distance > 0.0001:
                            delta_t = (ts - old_ts)
                            time_active += delta_t
                            time_categories[control_mode] += delta_t
                            speed = delta_distance / delta_t.to_sec()
                            if speed > max_speed:
                                max_speed = speed
                    old_position = vec
                    old_ts = ts
                elif topic == '/move_base/result':
                    control_mode = _MANUAL_CONTROL
                elif topic == '/move_base/goal':
                    control_mode = _AUTO_CONTROL

        total_time = (ts - ts_start).to_sec()
        active_time = time_active.to_sec()
        print("Driven distance: %r m" % distance_total)
        print("         Manual: %r m" % distance_categories[_MANUAL_CONTROL])
        print("           Auto: %r m" % distance_categories[_AUTO_CONTROL])
        print("Max speed: %r m/s" % max_speed)
        print("Average speed (active time): %r m/s" % (distance_total / active_time))
        print("Average speed (total time): %r m/s" % (distance_total / total_time))
        print("Active duration: %r s" % active_time)
        print("         Manual: %r s" % time_categories[_MANUAL_CONTROL].to_sec())
        print("           Auto: %r s" % time_categories[_AUTO_CONTROL].to_sec())
        print("Total duration: %r s" % total_time)
