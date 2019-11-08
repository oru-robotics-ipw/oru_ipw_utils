from __future__ import absolute_import, print_function

from .analysis import Analysis


class PeopleAnalysis(Analysis):
    """Analysis of SPENCER messages"""

    def analyse(self):
        people = set()
        for bag in self._bags:
            for topic, msg, t in bag.read_messages(
                    topics=('/spencer/perception/tracked_persons_confirmed_by_HOG_or_upper_body_or_moving',)):
                for track in msg.tracks:
                    people.add(track.track_id)

        print("Tracked persons: %r" % len(people))