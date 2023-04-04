#!/usr/bin/env python

import rospy
from any_msgs.msg import Float64Stamped
import subprocess
import re


def get_param(name):
    # Get the parameter, print an error if not available.
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        reason = 'Parameter ' + name + ' not found on server.'
        rospy.logerr(reason)
        rospy.signal_shutdown(reason)

if __name__ == '__main__':
    # Initialize the ROS node.
    rospy.init_node('pinger_ros')

    # Read the ROS parameters.
    target_ip = rospy.get_param('~target_ip', 'localhost')
    ping_rate = rospy.get_param('~ping_rate', 1)
    ping_topic = rospy.get_param('~ping_topic', 'ping')
    ping_timeout = rospy.get_param('~ping_timeout', 3)
    ping_packets = rospy.get_param('~ping_packets', 1)

    pinger_pub = rospy.Publisher(ping_topic, Float64Stamped, queue_size=10)

    rate = rospy.Rate(ping_rate)

    # Repeat until the node is shut down.
    while not rospy.is_shutdown():

      average_ping = 0
      response=''

      try:
          response = subprocess.check_output(
              ['ping', '-c', str(ping_packets), '-W', str(ping_timeout), target_ip],
              stderr=subprocess.STDOUT,  # get all output
              universal_newlines=True  # return string not bytes
          )
      except subprocess.CalledProcessError:
          # This happens if any packets are missed
          pass
          #print 'Failed to ping PC'
              
      time_indices = [ti.start() for ti in re.finditer(' time=', response)]
      ms_indices = [mi.start() for mi in re.finditer(' ms', response)]

      for time_idx, ms_idx in zip(time_indices, ms_indices):
        average_ping = average_ping + float(response[time_idx+6:ms_idx])

      # Add timeout time for missed packages
      average_ping = average_ping + 1000.0*ping_timeout*(ping_packets - len(time_indices))

      average_ping = average_ping / ping_packets / 1000.0
      #print 'Average Ping: ' + str(average_ping) + 'ms'

      # Create ping msg
      pingMsg = Float64Stamped()
      pingMsg.header.frame_id = target_ip
      pingMsg.header.stamp = rospy.Time.now()
      pingMsg.value = average_ping

      # Publish the ping and sleep.
      pinger_pub.publish(pingMsg)
      rate.sleep()

