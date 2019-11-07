from __future__ import absolute_import, print_function

import abc

import rosbag


class Analysis(object):
    """Base class of analysis types"""
    __metaclass__ = abc.ABCMeta

    def __init__(self, bag_files):
        """Constructor

        :type bag_files: List[Path]
        """
        self.bag_file_paths = bag_files

    @property
    def _bags(self):
        """Generator of bag files"""
        for p in self.bag_file_paths:
            yield rosbag.Bag(str(p))

    @abc.abstractmethod
    def analyse(self):
        """Implement the specific analysis step"""
        pass
