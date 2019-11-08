#!/usr/bin/env python2

from __future__ import absolute_import, print_function

import argparse
import time

from pathlib2 import Path

from .log_analysis import LogAnalysis
from .odom import OdomAnalysis
from .spencer import PeopleAnalysis

_ANALYSIS_MAP = {
    'odom': OdomAnalysis,
    'log': LogAnalysis,
    'people': PeopleAnalysis,
}


def main():
    """Main program entrypoint"""
    parser = argparse.ArgumentParser(description='Analyse speed and acceleration in a bag file')
    parser.add_argument('analysis',
                        choices=sorted(_ANALYSIS_MAP.keys()),
                        help="Type of analysis to perform")
    parser.add_argument('bag_files',
                        nargs='+',
                        type=Path,
                        help='Path to bag file')
    args = parser.parse_args()

    analysis = args.analysis  # type: str

    analyser = _ANALYSIS_MAP[analysis](args.bag_files)

    t_before = time.time()
    analyser.analyse()
    t_after = time.time()
    print("\nTime for analysis: %r s" % (t_after - t_before))
