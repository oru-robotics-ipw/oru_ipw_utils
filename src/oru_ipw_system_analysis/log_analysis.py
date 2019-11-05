"""Analyse /rosout (console log)"""

from __future__ import absolute_import, print_function

import abc
import inspect
import re
import sys

from rosgraph_msgs.msg import Log

from .analysis import Analysis


class BaseCounter(object):
    """Base counter object that can count occurrences of log messages"""

    def __init__(self, name):
        """Constructor

        :type name: str
        """
        self.count = 0
        self.name = name

    def __repr__(self):
        return "%s: %r" % (self.name, self.count)


class MatchCounterLevel(BaseCounter):
    """Counter based on log level"""

    def __init__(self, name, level):
        """Constructor

        :type name: str
        :type level: int
        """
        super(MatchCounterLevel, self).__init__(name)
        self.level = level

    def __call__(self, msg):
        """

        :type msg: Log
        :return:
        """
        if msg.level == self.level:
            self.count += 1
        # We still want warnings about unmatched items that happen to be at any level, so never return true
        return False


class MatchCounterLiteral(BaseCounter):
    """Counter using literal matches"""

    def __init__(self, name, match):
        """Constructor

        :type name: str
        :type match: str
        """
        super(MatchCounterLiteral, self).__init__(name)
        self.match = match

    def __call__(self, msg):
        """

        :type msg: Log
        :return:
        """
        if msg.msg == self.match:
            self.count += 1
            return True
        return False


class MatchCounterRegex(BaseCounter):
    """Counter using regex"""

    def __init__(self, name, match):
        """Constructor

        :type name: str
        :type match: str
        """
        super(MatchCounterRegex, self).__init__(name)
        self.match = re.compile(match)

    def __call__(self, msg):
        """

        :type msg: Log
        :return:
        """
        if self.match.match(msg.msg):
            self.count += 1
            return True
        return False


class BaseLogNodeAnalyser(object):
    """Base class to analyse messages from a specific node"""

    __metaclass__ = abc.ABCMeta

    def __init__(self, name):
        self.name = name

    @abc.abstractmethod
    def analyse(self, msg):
        """Analyse move_base messages

        :type msg: Log
        :return:
        """
        pass

    @abc.abstractmethod
    def summarize(self):
        """Print summary information"""
        print(self.name + ':')
        pass


class CountLogNodeAnalyser(BaseLogNodeAnalyser):
    """Count occurrences of messages from a specific node"""

    def __init__(self, name, matchers, verbose=True):
        """Constructor

        :type name: str
        :type matchers: list
        """
        super(CountLogNodeAnalyser, self).__init__(name)
        self.matchers = matchers
        self.matchers.append(MatchCounterLevel('Level: Warning', Log.WARN))
        self.matchers.append(MatchCounterLevel('Level: Errors', Log.ERROR))
        self.matchers.append(MatchCounterLevel('Level: Fatal', Log.FATAL))
        self.verbose = verbose

    def analyse(self, msg):
        """Analyse move_base messages

        :type msg: Log
        :return:
        """
        matched = False
        for matcher in self.matchers:
            if matcher(msg):
                matched = True
        if not matched and self.verbose:
            print("%s unmatched: %s" % (self.name, msg.msg))

    def summarize(self):
        """Print summary information"""
        printed_header = False
        for matcher in self.matchers:
            if matcher.count > 0:
                if not printed_header:
                    print(self.name + ':')
                    printed_header = True
                print('  %r' % matcher)


class MoveBaseAnalyser(CountLogNodeAnalyser):
    """Analyse log messages from move_base"""

    def __init__(self):
        super(MoveBaseAnalyser, self).__init__(
            '/move_base',
            [
                MatchCounterLiteral('TEB: Trajectory not feasible',
                                    'TebLocalPlannerROS: trajectory is not feasible. Resetting planner...'),
                MatchCounterRegex('TEB: Possible oscillation',
                                  '^TebLocalPlannerROS: possible oscillation'),
                MatchCounterLiteral('TEB: Oscillation recovery disabled/expired',
                                    'TebLocalPlannerROS: oscillation recovery disabled/expired.'),
                MatchCounterLiteral('Recovery: Rotate',
                                    'Rotate recovery behavior started.'),
                MatchCounterRegex('Recovery: Rotate recovery failure due to collision',
                                  "^Rotate recovery can't rotate in place because there is a potential collision\."),
                MatchCounterRegex('Recovery: Clearing costmap', '^Clearing costmap to unstuck robot'),
                MatchCounterRegex('Too slow: Control rate', '^Control loop missed its desired rate of '),
                MatchCounterRegex('Too slow: Map update rate', '^Map update loop missed its desired rate of '),
                MatchCounterRegex("Sensors: Laser failed deadline",
                                  '^The /sensors/laser/subsampled_points observation buffer has not been updated'),
                MatchCounterRegex("Sensors: Camera failed deadline",
                                  '^The /sensors/camera/subsampled_points observation buffer has not been updated'),
                MatchCounterRegex('Pose: Extrapolation into past',
                                  '^Extrapolation Error looking up robot pose: Lookup would require extrapolation into the past\.'),
                MatchCounterRegex('Pose: Lookup failure',
                                  '^Could not get robot pose, cancelling reconfiguration'),
                MatchCounterLiteral('Successful navigations', 'GOAL Reached!'),
                MatchCounterLiteral('Failed navigations',
                                    'Aborting because a valid control could not be found. Even after executing all recovery behaviors'),
                MatchCounterLiteral('Sensors: Missed update blocked navigation',
                                    "[/move_base]:Sensor data is out of date, we're not going to allow commanding of the base for safety"),
            ])


class MclAnalyser(CountLogNodeAnalyser):
    """Analyse log messages from QuickMCL"""

    def __init__(self):
        super(MclAnalyser, self).__init__(
            '/mcl/quickmcl_node',
            [
            ])


class ControllerAnalyser(CountLogNodeAnalyser):
    """Analyse log messages from Controller"""

    def __init__(self):
        super(ControllerAnalyser, self).__init__(
            '/oru_ipw_controller',
            [
                MatchCounterLiteral('Collision', 'Collision'),
                MatchCounterLiteral('Collision ended', 'Collision ended'),
            ])


class DriverAnalyser(CountLogNodeAnalyser):
    """Analyse log messages from HRP driver"""

    def __init__(self):
        super(DriverAnalyser, self).__init__(
            '/hrp/am_driver_safe',
            [
                MatchCounterLiteral('Collision', 'Collision'),
            ])


def _is_concrete_analyser(x):
    """Meta programming function to check if a class is a concrete analyser"""
    if not (inspect.isclass(x) and issubclass(x, BaseLogNodeAnalyser) and not inspect.isabstract(x)):
        return False
    if len(inspect.getargspec(x.__init__).args) > 1:
        return False
    return True


class LogAnalysis(Analysis):
    """Analysis of log messages"""

    def __init__(self, bag_file):
        super(LogAnalysis, self).__init__(bag_file)
        # Instantiate all concrete analysers in this module
        analysers = inspect.getmembers(sys.modules[__name__], _is_concrete_analyser)
        self._node_switch = {}
        for _, e in analysers:
            instance = e()
            self._node_switch[instance.name] = instance

    def analyse(self):
        """Implement the specific analysis step"""
        encountered = set()
        for topic, msg, t in self.bag.read_messages(topics=('/rosout',)):
            analyser = self._node_switch.get(msg.name, None)
            if analyser:
                analyser.analyse(msg)
            elif msg.name not in encountered:
                encountered.add(msg.name)
                print("No analyser for %s, doing generic analysis" % msg.name)
                self._node_switch[msg.name] = CountLogNodeAnalyser(msg.name, [], False)

        print()
        for analyser in self._node_switch.values():
            analyser.summarize()
