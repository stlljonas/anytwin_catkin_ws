#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
  @author: Enea Scioni, Yoshua Nava
  @email: <escioni@anybotics.com>, <ynava@anybotics.com>
  @affiliation: ANYbotics AG
  @date: 12/05/2020
  
Original version from any_topics_tools/remap.
"""
from __future__ import print_function

import roslib
import rospy
import rostopic

import argparse
import importlib
import sys


class TopicOp:

    def __init__(self):
        parser = argparse.ArgumentParser(
            formatter_class=argparse.RawTextHelpFormatter,
            description='Apply a Python operation to a topic.\n\n'
                        'A node is created that subscribes to a topic,\n'
                        'applies a Python expression to the topic (or topic\n'
                        'field) message \'m\', and publishes the result\n'
                        'through another topic.\n\n'
                        'Usage:\n\trosrun topic_tools remap '
                        '<input_topic> <output topic> <output type> '
                        '[<expression on m>] [--import numpy tf]\n\n'
                        'Example:\n\trosrun topic_tools remap /imu/orientation '
                        '/norm std_msgs/Float64 '
                        '\'sqrt(sum(array([m.x, m.y, m.z, m.w])))\'')
        parser.add_argument('input_topic', help='Input topic or topic field.')
        parser.add_argument('output_topic', help='Output topic.')
        parser.add_argument('output_frame', help='Output frame.')
        parser.add_argument('output_type', help='Output topic type.')
        parser.add_argument(
            '-i', '--import', dest='modules', nargs='+', default=['numpy'],
            help='List of Python modules to import.'
        )
        parser.add_argument(
            '--input-frame',
            help='Input frame.'
        )
        parser.add_argument(
            '--output-frame-is-prefix', action='store_true',
            help='Whether the output frame provided should be taken as a prefix or a full name.'
        )
        parser.add_argument(
            '--wait-for-start', action='store_true',
            help='Wait for input messages.'
        )
        parser.add_argument(
            '--latch', action='store_true',
            help='Set publisher to latched.'
        )
        parser.add_argument(
            '-qp','--queue-size-publisher', dest='pub_queue_size', action='store',
            help='Set publisher message queue.'
        )
        parser.add_argument(
            '-qs','--queue-size-subscriber', dest='sub_queue_size', action='store',
            help='Set subscriber message queue.'
        )

        # get and strip out ros args first
        argv = rospy.myargv()
        args = parser.parse_args(argv[1:])

        self.modules = {}
        for module in args.modules:
            try:
                mod = importlib.import_module(module)
            except ImportError:
                print('Failed to import module: %s' % module, file=sys.stderr)
            else:
                self.modules[module] = mod

        # Topics.
        input_topic_in_ns = args.input_topic
        if not input_topic_in_ns.startswith('/'):
            input_topic_in_ns = rospy.get_namespace() + args.input_topic
        input_topic_in_ns = args.input_topic

        input_class, input_topic, self.input_fn = rostopic.get_topic_class(
            input_topic_in_ns, blocking=args.wait_for_start)
        if input_topic is None:
            print('ERROR: Wrong input topic (or topic field): %s' % input_topic_in_ns, file=sys.stderr)
            sys.exit(1)

        output_topic = args.output_topic

        # Message types.
        self.output_class = roslib.message.get_message_class(args.output_type)

        # Frames.
        self.input_frame = args.input_frame
        self.output_frame_is_prefix = bool(args.output_frame_is_prefix)
        self.output_frame = args.output_frame

        # ROS communications.
        self.pub = rospy.Publisher(output_topic, self.output_class, queue_size=args.pub_queue_size, latch=args.latch)
        self.sub = rospy.Subscriber(input_topic, input_class, self.callback, queue_size=int(args.sub_queue_size))

    def callback(self, m):
        if self.input_fn is not None:
            m = self.input_fn(m)

        # Remap frame ID.
        if self.output_frame_is_prefix:
            m.header.frame_id = m.header.frame_id.replace(self.input_frame, self.output_frame)
        else:
            m.header.frame_id = self.output_frame

        # Re-publish.
        self.pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('remap', anonymous=True)
    app = TopicOp()
    rospy.spin()
