# coding=utf-8
from __future__ import print_function

import coverage
import os
import rospy

from argparse import ArgumentParser
from ros_pytest import run_pytest


def run_pytest_coverage(argv):
    # Parse arguments
    parser = ArgumentParser()
    parser.add_argument("--test", type=str, help="Path where the ROS tests are located.", required=True)
    parser.add_argument("--module", type=str, help="Python module to test (mostly equal to the package name)", required=True)
    parser.add_argument("--coverage_file", type=str, help="File to store coverage data.", required=True)

    args, other_args = parser.parse_known_args()

    # Set the test_module parameter used by ros_pytest
    rospy.set_param('test_module', args.test)

    # Run coverage
    cov = coverage.Coverage(source=[args.module], data_file=args.coverage_file)
    # cov.load() # Used to append to existing (Not required)
    cov.start()
    exit_code = run_pytest(other_args)
    cov.stop()
    cov.save()
    return exit_code
