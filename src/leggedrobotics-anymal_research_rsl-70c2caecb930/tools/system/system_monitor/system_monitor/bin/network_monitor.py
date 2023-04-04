#!/usr/bin/env python

from __future__ import print_function

import re
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from system_monitor_msgs.msg import NetworkStatus, NetworkInterfaceStatus, NetworkInterfaceOperationStatus, StatusCheck, MonitoringStatus

from system_monitor import *

KB_to_MB = 1.0 / 1024.0
B_to_MB = 1.0 / 1024.0 / 1024.0


def get_interface_names_and_traffics():
    stdout = get_command_stdout('ifstat -q -S 1 1')
    lines = stdout.splitlines()
    interface_names = lines[0].split()
    traffics = lines[3].split()  # lines[2] is empty.
    interface_names_and_traffics = []
    for i in range(0, len(interface_names)):
        interface_name = interface_names[i]
        incoming_traffic = KB_to_MB * float(traffics[2 * i])
        outgoing_traffic = KB_to_MB * float(traffics[2 * i + 1])
        interface_names_and_traffics.append(
            [interface_name, incoming_traffic, outgoing_traffic])
    return interface_names_and_traffics


def network_checks_list_to_dict(network_checks_list):
    network_checks_dict = {}
    for network_check in network_checks_list:
        interface_name = network_check['interface_name']
        del network_check['interface_name']
        network_checks_dict[interface_name] = network_check
    return network_checks_dict


def get_interface_info(interface_name, info_name):
    return get_file_content('/sys/class/net/' + interface_name + '/' + info_name)


def get_interface_statistics(interface_name, statistics_name):
    return get_file_content('/sys/class/net/' + interface_name + '/statistics/' + statistics_name)


def get_operation_status(interface_name):
    return get_interface_info(interface_name, 'operstate')


def get_mtu(interface_name):
    return int(get_interface_info(interface_name, 'mtu'))


def get_total_received_data(interface_name):
    return B_to_MB * float(get_interface_statistics(interface_name, 'rx_bytes'))


def get_total_transmitted_data(interface_name):
    return B_to_MB * float(get_interface_statistics(interface_name, 'tx_bytes'))


def get_num_collisions(interface_name):
    return int(get_interface_statistics(interface_name, 'collisions'))


def get_num_reception_errors(interface_name):
    return int(get_interface_statistics(interface_name, 'rx_errors'))


def get_num_transmission_errors(interface_name):
    return int(get_interface_statistics(interface_name, 'tx_errors'))


def operation_status_to_msg(operation_status):
    operation_status_map = {'up': NetworkInterfaceOperationStatus.UP,
                            'down': NetworkInterfaceOperationStatus.DOWN}
    return operation_status_map[operation_status]


def is_in_list(string, list_of_regex):
    for regex in list_of_regex:
        if re.search(regex, string):
            return True
    return False


class NetworkMonitor(Monitor):
    def __init__(self):
        Monitor.__init__(self, 'network_monitor', 'Network', NetworkStatus)

    def read_parameters(self):
        self.interface_name_whitelist = rospy.get_param(
            '~interface_name_whitelist', [])
        self.interface_name_blacklist = rospy.get_param(
            '~interface_name_blacklist', [])
        self.network_checks = network_checks_list_to_dict(
            rospy.get_param('~network_checks', []))

    def reset_msgs(self):
        self.diag_status.level = DiagnosticStatus.ERROR
        self.diag_status.message = 'N/A'
        self.diag_status.values = []
        self.status_msg.monitoring_status.data = MonitoringStatus.NA
        self.status_msg.interfaces = []

    def collect_data(self):
        return get_interface_names_and_traffics()

    def fill_msgs(self, interface_names_and_traffics):
        for interface_names_and_traffic in interface_names_and_traffics:
            interface_name = interface_names_and_traffic[0]

            # Skip if whitelist given and name is not part of it.
            if self.interface_name_whitelist and not is_in_list(interface_name, self.interface_name_whitelist):
                continue

            # Skip if blacklist given and name is part of it.
            if self.interface_name_blacklist and is_in_list(interface_name, self.interface_name_blacklist):
                continue

            # Get other data.
            operation_status = get_operation_status(interface_name)
            incoming_traffic = interface_names_and_traffic[1]
            outgoing_traffic = interface_names_and_traffic[2]
            mtu = get_mtu(interface_name)
            total_received_data = get_total_received_data(interface_name)
            total_transmitted_data = get_total_transmitted_data(interface_name)
            num_collisions = get_num_collisions(interface_name)
            num_reception_errors = get_num_reception_errors(interface_name)
            num_transmission_errors = get_num_transmission_errors(
                interface_name)

            # Fill status message.
            interface_status_msg = NetworkInterfaceStatus()
            interface_status_msg.name = interface_name
            interface_status_msg.operation_status.data = operation_status_to_msg(
                operation_status)
            interface_status_msg.operation_status_check.data = StatusCheck.NA
            interface_status_msg.incoming_traffic = incoming_traffic
            interface_status_msg.outgoing_traffic = outgoing_traffic
            interface_status_msg.traffic_check.data = StatusCheck.NA
            interface_status_msg.mtu = mtu
            interface_status_msg.total_received_data = total_received_data
            interface_status_msg.total_transmitted_data = total_transmitted_data
            interface_status_msg.num_collisions = num_collisions
            interface_status_msg.num_reception_errors = num_reception_errors
            interface_status_msg.num_transmission_errors = num_transmission_errors
            self.status_msg.interfaces.append(interface_status_msg)

            # Fill diagnostics message.
            self.diag_status.values.append(
                KeyValue('Interface Name', interface_name))
            self.diag_status.values.append(
                KeyValue('Operation Status', operation_status))
            self.diag_status.values.append(
                KeyValue('Incoming Traffic', '%.3f MB/s' % incoming_traffic))
            self.diag_status.values.append(
                KeyValue('Outgoing Traffic', '%.3f MB/s' % outgoing_traffic))
            self.diag_status.values.append(
                KeyValue('MTU', '%i B' % mtu))
            self.diag_status.values.append(
                KeyValue('Total Received Data', '%.3f MB' % total_received_data))
            self.diag_status.values.append(
                KeyValue('Total Transmitted Data', '%.3f MB' % total_transmitted_data))
            self.diag_status.values.append(
                KeyValue('Collision Count', str(num_collisions)))
            self.diag_status.values.append(
                KeyValue('Reception Error Count', str(num_reception_errors)))
            self.diag_status.values.append(
                KeyValue('Transmission Error Count', str(num_transmission_errors)))

    def perform_status_checks(self):
        max_diag_status_level = DiagnosticStatus.OK
        messages = []

        # Run all given network checks.
        for check_interface_name in self.network_checks:
            # Find the interface for the given network check.
            id = -1
            for i in range(0, len(self.status_msg.interfaces)):
                if check_interface_name == self.status_msg.interfaces[i].name:
                    id = i
            if id == -1:
                max_diag_status_level = get_max_diag_status_level(
                    max_diag_status_level, DiagnosticStatus.WARN)
                messages.append(
                    'Failed to check inexistent interface \'' + check_interface_name + '\'.')
                continue

            # Perform the checks for the given interface.
            interface = self.status_msg.interfaces[id]
            network_check = self.network_checks[interface.name]
            if network_check['check_operation_status'] and interface.operation_status.data != NetworkInterfaceOperationStatus.UP:
                max_diag_status_level = get_max_diag_status_level(
                    max_diag_status_level, DiagnosticStatus.ERROR)
                messages.append(
                    'The interface \'' + interface.name + '\' is not up.')
                self.status_msg.interfaces[id].operation_status_check.data = StatusCheck.ERROR
                self.status_msg.interfaces[id].traffic_check.data = StatusCheck.NA
            elif interface.incoming_traffic / network_check['max_traffic'] > network_check['traffic_error'] or \
                    interface.outgoing_traffic / network_check['max_traffic'] > network_check['traffic_error']:
                max_diag_status_level = get_max_diag_status_level(
                    max_diag_status_level, DiagnosticStatus.ERROR)
                messages.append(
                    'The interface \'' + interface.name + '\' traffic is too high.')
                self.status_msg.interfaces[id].operation_status_check.data = StatusCheck.OK
                self.status_msg.interfaces[id].traffic_check.data = StatusCheck.ERROR
            elif interface.incoming_traffic / network_check['max_traffic'] > network_check['traffic_warn'] or \
                    interface.outgoing_traffic / network_check['max_traffic'] > network_check['traffic_warn']:
                max_diag_status_level = get_max_diag_status_level(
                    max_diag_status_level, DiagnosticStatus.WARN)
                messages.append(
                    'The interface \'' + interface.name + '\' traffic is high.')
                self.status_msg.interfaces[id].operation_status_check.data = StatusCheck.OK
                self.status_msg.interfaces[id].traffic_check.data = StatusCheck.WARN
            else:
                self.status_msg.interfaces[id].operation_status_check.data = StatusCheck.OK
                self.status_msg.interfaces[id].traffic_check.data = StatusCheck.OK

        # Fill in diagnostic msg.
        self.diag_status.level = max_diag_status_level
        if not messages:
            self.diag_status.message = 'Ok'
        else:
            self.diag_status.message = ''
            for message in messages:
                self.diag_status.message = self.diag_status.message + message + ' '


def main():
    network_monitor = NetworkMonitor()
    network_monitor.run()


if __name__ == '__main__':
    main()
