#!/usr/bin/env python

import argparse
import progressbar
import rosbag

# This script can be used to fix the timestamps of messages contained in a ROS bag,
# in cases where the rosbag header timestamps are inconsistent, but the ROS message headers
# can be assumed to be correct (e.g. when recording data in an external computer which is not
# synchronized with the ROS master)

# Argument parsing.
parser = argparse.ArgumentParser()
parser.add_argument("input_path", help="File path of Input ROS bag.", type=str)
parser.add_argument("output_path", help="File path for Output ROS bag.", type=str)
args = parser.parse_args()

input_bag_file_path = args.input_path
output_bag_file_path = args.output_path

# Attempt to open intput bag.
print("Opening input bag...")
try:
  input_bag = rosbag.Bag(input_bag_file_path)
except:
  print("Error loading input bag: '" + input_bag_file_path + "' ")
  exit()

print("Input bag loaded.")
num_messages = input_bag.get_message_count()

# Progress bar init.
progress_bar = progressbar.ProgressBar(maxval=num_messages, \
    widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])

with rosbag.Bag(output_bag_file_path, mode='w', compression='lz4') as outbag:

  print("Correcting data timestamps...")

  msg_index = 0
  progress_bar.start()
  for topic, msg, t in input_bag.read_messages():
    msg_index += 1

    # If the message is headerless, ignore it. (Exception: actuator readings)
    if(msg._has_header == False and "actuator_readings" not in topic):
      continue

    # Special case: actuator readings.
    if(topic == "/anymal_lowlevel_controller/actuator_readings"):
      for i in range(0, len(msg.readings)):
        outbag.write(topic, msg, msg.readings[i].header.stamp)
      continue

    # Regular messages.
    if(msg._has_header):
      outbag.write(topic, msg, msg.header.stamp) # Header->topic time

    progress_bar.update(msg_index)

  outbag.close()

print("Timestamp correction finished successfully")
print("Output bag saved.")