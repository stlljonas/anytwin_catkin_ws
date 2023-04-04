#!/usr/bin/env python

import argparse
import progressbar
import rosbag

# This script can be used to rename the topics and frames of messages contained in a ROS bag,
# from 'brand names' such as <velodyne>, <realsense_d435> and <blackfly> to generic names, as
# <lidar>, <depth_camera> and <wide_angle_camera>.

# Argument parsing.
parser = argparse.ArgumentParser()
parser.add_argument("input_path", help="File path of Input ROS bag.", type=str)
parser.add_argument("output_path", help="File path for Output ROS bag.", type=str)
parser.add_argument('--verbose', help='Log more output into the terminal.', action='store_true')
args = parser.parse_args()

input_bag_file_path = args.input_path
output_bag_file_path = args.output_path

# Topic renaming map, can be changed to suit every user needs.
topic_renaming_map = {
  "blackfly" : "wide_angle_camera",
  "realsense/" : "depth_camera/",       # Special case, Point Cloud Processor output clouds (e.g. velodyne/realsense, quadruple_realsense)
  "realsense_d435" : "depth_camera",
  "velodyne" : "lidar",
}

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

  print("Renaming topics and frames...")

  msg_index = 0
  progress_bar.start()
  for topic, msg, t in input_bag.read_messages():
    msg_index += 1

    topic_matches = [prefix for prefix in topic_renaming_map.keys() if prefix in topic ]

    # Renaming.
    if len(topic_matches) > 0:
      # Log (if enabled).
      if(args.verbose):
        print("Match!")
        print(" Topic: " + topic)
        print(" Frame: " + msg.header.frame_id)

      # Iterate over all matched topic names.
      for topic_match in topic_matches:
        # Rename topic.
        topic = topic.replace(topic_match, topic_renaming_map[topic_match])

        # Rename frame if needed.
        if(topic_match in msg.header.frame_id):
          msg.header.frame_id = msg.header.frame_id.replace(topic_match, topic_renaming_map[topic_match])

    # Write msg to bag.
    outbag.write(topic, msg, t)

    progress_bar.update(msg_index)

  outbag.close()

print("Topic and frame renaming finished successfully")
print("Output bag saved.")