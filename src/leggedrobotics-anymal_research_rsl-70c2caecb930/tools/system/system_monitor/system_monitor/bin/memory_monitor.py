#!/usr/bin/env python

from __future__ import print_function

import re
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from system_monitor_msgs.msg import MemoryStatus, MemoryTypeStatus, StatusCheck, MonitoringStatus

from system_monitor import *


def get_memory_data():
    stdout = get_command_stdout('free -t --mega')
    lines = stdout.splitlines()
    values = []
    for line in lines[1:]:  # Ignore the title.
        # print(line)
        line_values = line.split()
        values.append(line_values[1:])   # Ignore the title line.
    return values


class MemoryMonitor(Monitor):
    def __init__(self):
        Monitor.__init__(self, 'memory_monitor', 'Memory', MemoryStatus)

    def read_parameters(self):
        self.physical_memory_usage_warn = rospy.get_param(
            '~physical_memory_usage_warn', 0.80)
        self.physical_memory_usage_error = rospy.get_param(
            '~physical_memory_usage_error', 0.95)

    def reset_msgs(self):
        self.diag_status.level = DiagnosticStatus.ERROR
        self.diag_status.message = 'N/A'
        self.diag_status.values = []
        self.status_msg = MemoryStatus()
        self.status_msg.monitoring_status.data = MonitoringStatus.NA
        self.status_msg.physical_usage_check.data = StatusCheck.NA

    def collect_data(self):
        return get_memory_data()

    def fill_msgs(self, memory_data):
        self.status_msg.physical.total = int(memory_data[0][0])
        self.status_msg.physical.used = int(memory_data[0][1])
        self.status_msg.physical.free = int(memory_data[0][2])
        self.status_msg.physical.usage = float(
            self.status_msg.physical.used)/float(self.status_msg.physical.total)

        self.status_msg.swap.total = int(memory_data[1][0])
        self.status_msg.swap.used = int(memory_data[1][1])
        self.status_msg.swap.free = int(memory_data[1][2])
        self.status_msg.swap.usage = float(
            self.status_msg.swap.used)/float(self.status_msg.swap.total)

        self.status_msg.total.total = int(memory_data[2][0])
        self.status_msg.total.used = int(memory_data[2][1])
        self.status_msg.total.free = int(memory_data[2][2])
        self.status_msg.total.usage = float(
            self.status_msg.total.used)/float(self.status_msg.total.total)

        self.diag_status.values.append(KeyValue(
            'Total Physical Memory', '%i MB' % self.status_msg.physical.total))
        self.diag_status.values.append(KeyValue(
            'Used Physical Memory', '%i MB' % self.status_msg.physical.used))
        self.diag_status.values.append(KeyValue(
            'Free Physical Memory', '%i MB' % self.status_msg.physical.free))
        self.diag_status.values.append(KeyValue(
            'Physical Memory Usage', '%.0f' % (100.0 * self.status_msg.physical.usage) + '%'))

        self.diag_status.values.append(KeyValue(
            'Total Swap Memory', '%i MB' % self.status_msg.swap.total))
        self.diag_status.values.append(KeyValue(
            'Used Swap Memory', '%i MB' % self.status_msg.swap.used))
        self.diag_status.values.append(KeyValue(
            'Free Swap Memory', '%i MB' % self.status_msg.swap.free))
        self.diag_status.values.append(KeyValue(
            'Swap Memory Usage', '%.0f' % (100.0 * self.status_msg.swap.usage) + '%'))

        self.diag_status.values.append(KeyValue(
            'Total Memory', '%i MB' % self.status_msg.total.total))
        self.diag_status.values.append(KeyValue(
            'Used Memory', '%i MB' % self.status_msg.total.used))
        self.diag_status.values.append(KeyValue(
            'Free Memory', '%i MB' % self.status_msg.total.free))
        self.diag_status.values.append(KeyValue(
            'Memory Usage', '%.0f' % (100.0 * self.status_msg.total.usage) + '%'))

    def perform_status_checks(self):
        if self.status_msg.physical.usage > self.physical_memory_usage_error:
            self.diag_status.level = DiagnosticStatus.ERROR
            self.diag_status.message = 'Physical memory usage is too high.'
            self.status_msg.physical_usage_check.data = StatusCheck.ERROR
        elif self.status_msg.physical.usage > self.physical_memory_usage_warn:
            self.diag_status.level = DiagnosticStatus.WARN
            self.diag_status.message = 'Physical memory usage is high.'
            self.status_msg.physical_usage_check.data = StatusCheck.WARN
        else:
            self.diag_status.level = DiagnosticStatus.OK
            self.diag_status.message = 'Ok'
            self.status_msg.physical_usage_check.data = StatusCheck.OK


def main():
    memory_monitor = MemoryMonitor()
    memory_monitor.run()


if __name__ == '__main__':
    main()
