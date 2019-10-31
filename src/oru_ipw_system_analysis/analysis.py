from __future__ import absolute_import, print_function

import abc

import rosbag


class Analysis(object):
    """Base class of analysis types"""
    __metaclass__ = abc.ABCMeta

    def __init__(self, bag_file):
        """Constructor

        :type bag_file: Path
        """
        self.bag_file_path = bag_file
        self.bag = rosbag.Bag(str(self.bag_file_path))

    @abc.abstractmethod
    def analyse(self):
        """Implement the specific analysis step"""
        pass
