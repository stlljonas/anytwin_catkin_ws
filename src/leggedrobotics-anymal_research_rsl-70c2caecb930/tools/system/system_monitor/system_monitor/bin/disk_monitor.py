#!/usr/bin/env python

from __future__ import print_function

import re
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from system_monitor_msgs.msg import DiskStatus, SingleDiskStatus, StatusCheck, MonitoringStatus

from system_monitor import *

MB_to_GB = 1.0 / 1024.0


def get_disks():
    stdout = get_command_stdout('df -Pmt ext4')
    lines = stdout.splitlines()
    disks = []
    for line in lines[1:]:  # Ignore the title.
        disks.append(line.split())
    return disks


def parse_size(string):
    return MB_to_GB * float(string)


def parse_percentage(string):
    number = re.search('(.+)%', string).group(1)
    return 0.01 * float(number)


class DiskMonitor(Monitor):
    def __init__(self):
        Monitor.__init__(self, 'disk_monitor', 'Disk', DiskStatus)

    def read_parameters(self):
        self.disk_usage_warn = rospy.get_param('~disk_usage_warn', 0.80)
        self.disk_usage_error = rospy.get_param('~disk_usage_error', 0.99)

    def reset_msgs(self):
        self.diag_status.level = DiagnosticStatus.ERROR
        self.diag_status.message = 'N/A'
        self.diag_status.values = []
        self.status_msg.monitoring_status.data = MonitoringStatus.NA
        self.status_msg.disks = []

    def collect_data(self):
        return get_disks()

    def fill_msgs(self, disks):
        for id in range(0, len(disks)):
            single_disk_status_msg = SingleDiskStatus()
            single_disk_status_msg.name = disks[id][0]
            single_disk_status_msg.size = parse_size(disks[id][1])
            single_disk_status_msg.used = parse_size(disks[id][2])
            single_disk_status_msg.available = parse_size(disks[id][3])
            single_disk_status_msg.usage = parse_percentage(disks[id][4])
            single_disk_status_msg.usage_check.data = StatusCheck.NA
            single_disk_status_msg.mounting_point = disks[id][5]
            self.status_msg.disks.append(single_disk_status_msg)

            self.diag_status.values.append(KeyValue(
                'Disk %d Name' % id, single_disk_status_msg.name))
            self.diag_status.values.append(KeyValue(
                'Disk %d Size' % id, '%.3f GB' % single_disk_status_msg.size))
            self.diag_status.values.append(KeyValue(
                'Disk %d Used' % id, '%.3f GB' % single_disk_status_msg.used))
            self.diag_status.values.append(KeyValue(
                'Disk %d Available' % id, '%.3f GB' % single_disk_status_msg.available))
            self.diag_status.values.append(KeyValue(
                'Disk %d Usage' % id, '%.0f' % (100.0 * single_disk_status_msg.usage) + '%'))
            self.diag_status.values.append(KeyValue(
                'Disk %d Mount Point' % id, single_disk_status_msg.mounting_point))

    def perform_status_checks(self):
        max_diag_status_level = DiagnosticStatus.OK
        messages = []

        # Check disk usage.
        for id in range(0, len(self.status_msg.disks)):
            if self.status_msg.disks[id].usage > self.disk_usage_error:
                max_diag_status_level = get_max_diag_status_level(
                    max_diag_status_level, DiagnosticStatus.ERROR)
                messages.append('Disk %d usage is too high.' % id)
                self.status_msg.disks[id].usage_check.data = StatusCheck.ERROR
            elif self.status_msg.disks[id].usage > self.disk_usage_warn:
                max_diag_status_level = get_max_diag_status_level(
                    max_diag_status_level, DiagnosticStatus.WARN)
                messages.append('Disk %d usage is high.' % id)
                self.status_msg.disks[id].usage_check.data = StatusCheck.WARN
            else:
                self.status_msg.disks[id].usage_check.data = StatusCheck.OK

        # Fill in diagnostic msg.
        self.diag_status.level = max_diag_status_level
        if not messages:
            self.diag_status.message = 'Ok'
        else:
            self.diag_status.message = ''
            for message in messages:
                self.diag_status.message = self.diag_status.message + message + ' '


def main():
    disk_monitor = DiskMonitor()
    disk_monitor.run()


if __name__ == '__main__':
    main()
