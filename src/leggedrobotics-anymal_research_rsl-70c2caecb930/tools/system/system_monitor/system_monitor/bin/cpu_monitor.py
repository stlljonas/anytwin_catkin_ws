#!/usr/bin/env python

from __future__ import print_function

import math
import re
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from system_monitor_msgs.msg import CpuStatus, CpuCoreStatus, CpuUtilizationStatus, StatusCheck, MonitoringStatus

from system_monitor import *


def get_clock_speeds():
    string = get_file_content('/proc/cpuinfo')
    regex = r'cpu MHz(.+): (.+)'
    matches = re.findall(regex, string)
    clock_speeds = []
    for match in matches:
        clock_speeds.append(float(match[1]))
    return clock_speeds


def get_load_averages():
    string = get_command_stdout('uptime')
    regex = r'(.+)load average: (.+), (.+), (.+)'
    matches = re.findall(regex, string)
    load_averages = []
    for id in range(1, 4):
        load_averages.append(float(matches[0][id].replace(',', '.')))
    return load_averages


def get_temperatures():
    string = get_command_stdout('sensors')
    regex = r'Core (.):(.+)\xc2\xb0C (.+)'
    matches = re.findall(regex, string)
    temperatures = []
    for match in matches:
        temperatures.append(float(match[1]))
    return temperatures


def get_utilizations():
    string = get_command_stdout('mpstat -P ALL 1 1')
    regex = r'Average:(.+)'
    matches = re.findall(regex, string)
    utilizations = []
    for match in matches[1:]:  # Ignore title.
        values = []
        values_str = match.split()
        for value_str in values_str[1:]:  # Ignore first value.
            values.append(0.01 * float(value_str.replace(',', '.')))
        utilizations.append(values)
    return utilizations


class CpuMonitor(Monitor):
    def __init__(self):
        Monitor.__init__(self, 'cpu_monitor', 'CPU', CpuStatus)

    def read_parameters(self):
        self.cpu_usage_warn = rospy.get_param('~cpu_usage_warn', 0.80)
        self.cpu_usage_error = rospy.get_param('~cpu_usage_error', 0.95)
        self.cpu_temperature_warn = rospy.get_param(
            '~cpu_temperature_warn', 90.0)
        self.cpu_temperature_error = rospy.get_param(
            '~cpu_temperature_error', 100.0)

    def reset_msgs(self):
        self.diag_status.level = DiagnosticStatus.ERROR
        self.diag_status.message = 'N/A'
        self.diag_status.values = []
        self.status_msg.monitoring_status.data = MonitoringStatus.NA
        self.status_msg.usage_check.data = StatusCheck.NA
        self.status_msg.cores = []

    def collect_data(self):
        clock_speeds = get_clock_speeds()
        load_averages = get_load_averages()
        temperatures = get_temperatures()
        utilizations = get_utilizations()
        return [clock_speeds, load_averages, temperatures, utilizations]

    def fill_msgs(self, data):
        [clock_speeds, load_averages, temperatures, utilizations] = data
        self.status_msg.utilization.user = utilizations[0][0]
        self.status_msg.utilization.nice = utilizations[0][1]
        self.status_msg.utilization.sys = utilizations[0][2]
        self.status_msg.utilization.iowait = utilizations[0][3]
        self.status_msg.utilization.irq = utilizations[0][4]
        self.status_msg.utilization.soft = utilizations[0][5]
        self.status_msg.utilization.steal = utilizations[0][6]
        self.status_msg.utilization.guest = utilizations[0][7]
        self.status_msg.utilization.gnice = utilizations[0][8]
        self.status_msg.utilization.idle = utilizations[0][9]
        self.status_msg.utilization.usage = self.status_msg.utilization.user + \
            self.status_msg.utilization.nice

        for id in range(0, len(clock_speeds)):
            core_status = CpuCoreStatus()
            core_status.clock_speed = clock_speeds[id]
            try:
                core_status.temperature = temperatures[id / 2]
            except:
                core_status.temperature = float('NaN')
            core_status.temperature_check.data = StatusCheck.NA
            core_status.utilization.user = utilizations[id][0]
            core_status.utilization.nice = utilizations[id][1]
            core_status.utilization.sys = utilizations[id][2]
            core_status.utilization.iowait = utilizations[id][3]
            core_status.utilization.irq = utilizations[id][4]
            core_status.utilization.soft = utilizations[id][5]
            core_status.utilization.steal = utilizations[id][6]
            core_status.utilization.guest = utilizations[id][7]
            core_status.utilization.gnice = utilizations[id][8]
            core_status.utilization.idle = utilizations[id][9]
            core_status.utilization.usage = core_status.utilization.user + \
                core_status.utilization.nice
            self.status_msg.cores.append(core_status)

        self.status_msg.load_average_1min = load_averages[0]
        self.status_msg.load_average_5min = load_averages[1]
        self.status_msg.load_average_15min = load_averages[2]

        self.diag_status.values.append(KeyValue(
            'Overall Utilization Usage', '%.0f' % (100.0 * self.status_msg.utilization.usage) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization User', '%.0f' % (100.0 * self.status_msg.utilization.user) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization Nice', '%.0f' % (100.0 * self.status_msg.utilization.nice) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization System', '%.0f' % (100.0 * self.status_msg.utilization.sys) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization IO Wait', '%.0f' % (100.0 * self.status_msg.utilization.iowait) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization IRQ', '%.0f' % (100.0 * self.status_msg.utilization.irq) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization Soft', '%.0f' % (100.0 * self.status_msg.utilization.soft) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization Steal', '%.0f' % (100.0 * self.status_msg.utilization.steal) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization Guest', '%.0f' % (100.0 * self.status_msg.utilization.guest) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization Guest Nice', '%.0f' % (100.0 * self.status_msg.utilization.gnice) + '%'))
        self.diag_status.values.append(KeyValue(
            'Overall Utilization Idle', '%.0f' % (100.0 * self.status_msg.utilization.idle) + '%'))

        for id in range(0, len(self.status_msg.cores)):
            self.diag_status.values.append(KeyValue(
                'Core %i Utilization Usage' % id, '%.0f' % (100.0 * self.status_msg.cores[id].utilization.usage) + '%'))

        for id in range(0, len(self.status_msg.cores)):
            self.diag_status.values.append(KeyValue(
                'Core %i Clock Speed' % id, '%.0f MHz' % self.status_msg.cores[id].clock_speed))

        for id in range(0, len(self.status_msg.cores)):
            self.diag_status.values.append(KeyValue(
                'Core %i Temperature' % id, '%.0f \xc2\xb0C' % self.status_msg.cores[id].temperature))

        self.diag_status.values.append(KeyValue(
            'Load Average (1 min)', '%.2f' % self.status_msg.load_average_1min))
        self.diag_status.values.append(KeyValue(
            'Load Average (5 min)', '%.2f' % self.status_msg.load_average_5min))
        self.diag_status.values.append(KeyValue(
            'Load Average (15 min)', '%.2f' % self.status_msg.load_average_15min))

    def perform_status_checks(self):
        max_diag_status_level = DiagnosticStatus.OK
        messages = []

        # Check CPU usage.
        if self.status_msg.utilization.usage > self.cpu_usage_error:
            max_diag_status_level = get_max_diag_status_level(
                max_diag_status_level, DiagnosticStatus.ERROR)
            messages.append('CPU usage is too high.')
            self.status_msg.usage_check.data = StatusCheck.ERROR
        elif self.status_msg.utilization.usage > self.cpu_usage_warn:
            max_diag_status_level = get_max_diag_status_level(
                max_diag_status_level, DiagnosticStatus.WARN)
            messages.append('CPU usage is high.')
            self.status_msg.usage_check.data = StatusCheck.WARN
        else:
            self.status_msg.usage_check.data = StatusCheck.OK

        # Check CPU temperatures.
        temperature_nan = False
        temperature_warn = False
        temperature_error = False
        for id in range(0, len(self.status_msg.cores)):
            if self.status_msg.cores[id].temperature > self.cpu_temperature_error:
                temperature_error = True
                self.status_msg.cores[id].temperature_check.data = StatusCheck.ERROR
            elif self.status_msg.cores[id].temperature > self.cpu_temperature_warn:
                temperature_warn = True
                self.status_msg.cores[id].temperature_check.data = StatusCheck.WARN
            elif math.isnan(self.status_msg.cores[id].temperature):
                temperature_nan = True
                self.status_msg.cores[id].temperature_check.data = StatusCheck.WARN
            else:
                self.status_msg.cores[id].temperature_check.data = StatusCheck.OK
        if temperature_error:
            max_diag_status_level = get_max_diag_status_level(
                max_diag_status_level, DiagnosticStatus.ERROR)
            messages.append('CPU temperature is too high.')
        elif temperature_warn:
            max_diag_status_level = get_max_diag_status_level(
                max_diag_status_level, DiagnosticStatus.WARN)
            messages.append('CPU temperature is high.')
        elif temperature_nan:
            max_diag_status_level = get_max_diag_status_level(
                max_diag_status_level, DiagnosticStatus.WARN)
            messages.append('CPU temperature cannot be monitored.')

        # Fill in diagnostic msg.
        self.diag_status.level = max_diag_status_level
        if not messages:
            self.diag_status.message = 'Ok'
        else:
            self.diag_status.message = ''
            for message in messages:
                self.diag_status.message = self.diag_status.message + message + ' '


def main():
    cpu_monitor = CpuMonitor()
    cpu_monitor.run()


if __name__ == '__main__':
    main()
