#!/usr/bin/env python

from __future__ import print_function

import abc
import rospy
import socket
import traceback

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from system_monitor_msgs.msg import MonitoringStatus

from system_monitor import *


class Monitor():
    __metaclass__ = abc.ABCMeta

    def __init__(self, node_name, status_name, status_type):
        rospy.init_node(node_name)
        self.diag_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=100)
        self.status_pub = rospy.Publisher(
            '~status', status_type, latch=False, queue_size=100)

        # Parameters and inputs.
        self.status_name = status_name
        self.hostname = socket.gethostname()
        self.update_frequency = rospy.get_param('~update_frequency', 1.0)
        self.read_parameters()

        # Initialize messages.
        self.status_msg = status_type()
        self.diag_status = DiagnosticStatus()
        self.diag_status.hardware_id = self.hostname
        self.diag_status.name = self.status_name + ' Status (' + self.hostname + ')'

    def reset_msgs(self):
        self.diag_status.level = DiagnosticStatus.ERROR
        self.diag_status.message = 'N/A'
        self.diag_status.values = []
        self.status_msg.monitoring_status.data = MonitoringStatus.NA
        self.status_msg.cores = []

    @abc.abstractmethod
    def read_parameters(self):
        pass

    @abc.abstractmethod
    def collect_data(self):
        pass

    @abc.abstractmethod
    def fill_msgs(self, data):
        pass

    @abc.abstractmethod
    def perform_status_checks(self):
        pass

    def run_analysis(self):
        try:
            # Reset messages.
            self.reset_msgs()

            # Collect data.
            data = self.collect_data()

            # Fill the messages.
            self.fill_msgs(data)

            # Perform status checks.
            self.perform_status_checks()

            # At this point, monitoring succeeded.
            self.status_msg.monitoring_status.data = MonitoringStatus.OK

        except Exception as exception:
            # Reset messages as they might have been filled partially.
            self.reset_msgs()
            self.status_msg.monitoring_status.data = MonitoringStatus.ERROR
            self.diag_status.message = 'Failed to monitor ' + \
                self.status_name + ': Exception: ' + str(exception)
            rospy.logdebug(traceback.format_exc())

    def publish_diag(self):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.get_rostime()
        diag_msg.status = [self.diag_status]
        self.diag_pub.publish(diag_msg)

    def publish_status(self):
        self.status_msg.stamp = rospy.get_rostime()
        self.status_pub.publish(self.status_msg)

    def run(self):
        # Periodically run analysis and publish outcome.
        rate = rospy.Rate(self.update_frequency)
        while not rospy.is_shutdown():
            # Analyze.
            self.run_analysis()

            # Publish messages.
            self.publish_diag()
            self.publish_status()

            # Sleep.
            rate.sleep()
