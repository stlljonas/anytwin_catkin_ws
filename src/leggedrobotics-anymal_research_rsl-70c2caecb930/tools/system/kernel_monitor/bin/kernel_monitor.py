#!/usr/bin/env python

from __future__ import print_function

import argparse
import json
import rospy
import subprocess

from datetime import datetime


log_level_str_to_ros = {'debug': rospy.DEBUG,
                        'info': rospy.INFO,
                        'warn': rospy.WARN,
                        'error': rospy.ERROR,
                        'fatal': rospy.FATAL}


def execute_continuous_command(cmd):
    popen = subprocess.Popen(
        cmd.split(), stdout=subprocess.PIPE, universal_newlines=True)
    for output in iter(popen.stdout.readline, ''):
        yield output.rstrip()
    popen.stdout.close()
    popen.wait()


def level_to_string(level):
    if level == 7:
        return 'DEBUG'
    elif level == 6 or level == 5:
        return 'INFO'
    elif level == 4:
        return 'WARN'
    elif level == 3:
        return 'ERROR'
    elif level == 2 or level == 1 or level == 0:
        return 'FATAL'


def timestamp_to_string(timestamp):
    d = datetime.fromtimestamp(timestamp*1e-6)
    return d.strftime("%Y-%m-%d %H:%M:%S")


def process_output(output):
    # The output is in the format:
    # {
    #   "__CURSOR" : "s=b43218f49c1a4c1097ba480fd20e39df;i=937be;b=6181dc48c38142b28f911ead52591d61;m=2dfa3dff1;t=5a9d58a95cb96;x=4c6db6560527a843",
    #   "__REALTIME_TIMESTAMP" : "1594109501754262",
    #   "__MONOTONIC_TIMESTAMP" : "12341993457",
    #   "_BOOT_ID" : "6181dc48c38142b28f911ead52591d61",
    #   "PRIORITY" : "6",
    #   "_MACHINE_ID" : "484978f119a4b94cbcab29c85d9854c2",
    #   "_HOSTNAME" : "thinkpad",
    #   "_TRANSPORT" : "kernel",
    #   "SYSLOG_FACILITY" : "0",
    #   "SYSLOG_IDENTIFIER" : "kernel",
    #   "_KERNEL_SUBSYSTEM" : "usb",
    #   "_KERNEL_DEVICE" : "+usb:7-2.4",
    #   "_SOURCE_MONOTONIC_TIMESTAMP" : "12341695312",
    #   "MESSAGE" : "usb 7-2.4: new high-speed USB device number 13 using xhci_hcd"
    # }
    # We need to determine the level:
    try:
        output_json = json.loads(output)
        level = int(output_json['PRIORITY'])
        timestamp_us = int(output_json['__REALTIME_TIMESTAMP'])
        message = '[' + level_to_string(level) + '] ' \
                  '[' + timestamp_to_string(timestamp_us) + '] ' \
                  '[' + output_json['_HOSTNAME'] + ']: ' + output_json['MESSAGE']
        if level == 7:
            rospy.logdebug(message)
            return
        elif level == 6 or level == 5:
            rospy.loginfo(message)
            return
        elif level == 4:
            rospy.logwarn(message)
            return
        elif level == 3:
            rospy.logerr(message)
            return
        elif level == 2 or level == 1 or level == 0:
            rospy.logfatal(message)
            return
    except Exception as exception:
        print(exception)
    rospy.logwarn('Failed to determine level of dmesg output: ' + output)


def monitor_kernel():
    cmd = 'journalctl --dmesg --follow --no-tail --output=json --directory=/var/log/journal'
    for output in execute_continuous_command(cmd):
        process_output(output)


def create_node(ros_log_level):
    rospy.init_node('kernel_monitor', log_level=ros_log_level)
    monitor_kernel()


def main():
    # As the ROS log level must be known at node creation (and cannot be changed afterwards),
    # it cannot be a ROS parameter as reading it requires the node to exist already.
    parser = argparse.ArgumentParser(
        description='ROS node monitoring the kernel.')
    parser.add_argument('--log-level', dest='log_level', default='info',
                        help='Log level the ROS node: debug, info, warn, error, fatal.')
    # ROS adds some additional arguments (node name, log file, etc.).
    # rospy.myargv()[1:] gets rid of them.
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        ros_log_level = log_level_str_to_ros[args.log_level]
    except:
        print('Failed to parse log level: ' + args.log_level)
        return
    create_node(ros_log_level)


if __name__ == '__main__':
    main()
