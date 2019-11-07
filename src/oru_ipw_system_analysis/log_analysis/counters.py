"""Classes that count occurrences of log messages"""

import re


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
