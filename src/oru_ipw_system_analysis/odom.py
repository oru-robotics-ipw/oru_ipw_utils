from __future__ import absolute_import, print_function

import numpy as np
import rospy

from .analysis import Analysis


class OdomAnalysis(Analysis):
    """Odometry analysis"""

    def analyse(self):
        old_position = None
        old_ts = None
        ts_start = None
        distance_total = 0.0
        max_speed = 0.0
        time_active = rospy.Duration(0)
        for topic, msg, t in self.bag.read_messages(topics=('/hrp/odom',)):
            assert hasattr(msg, 'header')
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
                    # Compute active time
                    if delta_distance > 0.0001:
                        delta_t = (ts - old_ts)
                        time_active += delta_t
                        speed = delta_distance / delta_t.to_sec()
                        if speed > max_speed:
                            max_speed = speed
                old_position = vec
                old_ts = ts

        total_time = (ts - ts_start).to_sec()
        active_time = time_active.to_sec()
        print("Driven distance: %r m" % distance_total)
        print("Max speed: %r m/s" % max_speed)
        print("Average speed (active time): %r m/s" % (distance_total / active_time))
        print("Average speed (total time): %r m/s" % (distance_total / total_time))
        print("Active duration: %r s" % active_time)
        print("Total duration: %r s" % total_time)
