#!/bin/bash

# Print the output of `rosbag info` into a file for each bag file in the current folder and its subfolders.
# Usage 1: ./create_rosbag_info_files.sh
# Usage 2: rosrun anymal_logging create_rosbag_info_files.sh
for i in $(find . -name '*.bag'); do
  rosbag info $i > "${i::-4}_info.txt"
done
