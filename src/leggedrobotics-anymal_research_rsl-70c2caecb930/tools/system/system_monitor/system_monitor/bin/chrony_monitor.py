#!/usr/bin/env python

from __future__ import print_function

import re
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from system_monitor_msgs.msg import ChronyStatus, ChronySyncStatus, StatusCheck, MonitoringStatus

from system_monitor import *


def split_line(line):
    colon_idx = line.find(':')
    return [line[:colon_idx].strip(), line[colon_idx+1:].strip()]


def get_chrony_status():
    stdout = get_command_stdout('chronyc tracking')
    lines = stdout.splitlines()
    split_lines = []
    for line in lines:
        split_lines.append(split_line(line))
    return split_lines


def parse_int(string):
    number = re.search('(.+)', string).group(1)
    return int(number)


def parse_sec(string):
    number = re.search('(.+) seconds', string).group(1)
    return float(number)


def parse_ppm(string):
    number = re.search('(.+) ppm', string).group(1)
    return float(number)


class ChronyMonitor(Monitor):
    def __init__(self):
        Monitor.__init__(self, 'chrony_monitor',
                         'Chrony Time Sync', ChronyStatus)

    def read_parameters(self):
        self.time_offset_warn = rospy.get_param('~time_offset_warn', 1e-5)
        self.time_offset_error = rospy.get_param('~time_offset_error', 1e-3)

    def reset_msgs(self):
        self.diag_status.level = DiagnosticStatus.ERROR
        self.diag_status.message = 'N/A'
        self.diag_status.values = []
        self.status_msg.monitoring_status.data = MonitoringStatus.NA
        self.status_msg.time_sync_status.data = ChronySyncStatus.NA
        self.status_msg.time_offset_check.data = StatusCheck.NA

    def collect_data(self):
        return get_chrony_status()

    def fill_msgs(self, split_lines):
        self.status_msg.reference_id = split_lines[0][1]
        self.status_msg.stratum = parse_int(split_lines[1][1])
        self.status_msg.reference_time = split_lines[2][1]
        self.status_msg.system_time = parse_sec(split_lines[3][1])
        self.status_msg.last_offset = parse_sec(split_lines[4][1])
        self.status_msg.rms_offset = parse_sec(split_lines[5][1])
        self.status_msg.frequency = parse_ppm(split_lines[6][1])
        self.status_msg.residual_frequency = parse_ppm(split_lines[7][1])
        self.status_msg.skew = parse_ppm(split_lines[8][1])
        self.status_msg.root_delay = parse_sec(split_lines[9][1])
        self.status_msg.root_dispersion = parse_sec(split_lines[10][1])
        self.status_msg.update_interval = parse_sec(split_lines[11][1])
        self.status_msg.leap_status = split_lines[12][1]

        for split_line in split_lines:
            self.diag_status.values.append(
                KeyValue(split_line[0], split_line[1]))

    def perform_status_checks(self):
        # Check if chrony is synching.
        is_running = self.status_msg.update_interval > 0.0
        if not is_running:
            self.diag_status.message = 'Chrony is not synchronizing.'
            self.status_msg.time_sync_status.data = ChronySyncStatus.INACTIVE
            return
        self.status_msg.time_sync_status.data = ChronySyncStatus.ACTIVE

        # Check the time offset.
        time_offset = self.status_msg.system_time
        if (abs(time_offset) > self.time_offset_error):
            self.diag_status.level = DiagnosticStatus.ERROR
            self.diag_status.message = 'Time offset is too high.'
            self.status_msg.time_offset_check.data = StatusCheck.ERROR
        elif (abs(time_offset) > self.time_offset_warn):
            self.diag_status.level = DiagnosticStatus.WARN
            self.diag_status.message = 'Time offset is high.'
            self.status_msg.time_offset_check.data = StatusCheck.WARN
        else:
            self.diag_status.level = DiagnosticStatus.OK
            self.diag_status.message = 'Ok'
            self.status_msg.time_offset_check.data = StatusCheck.OK


def main():
    chrony_monitor = ChronyMonitor()
    chrony_monitor.run()


if __name__ == '__main__':
    main()
