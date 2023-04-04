#!/bin/bash

# Setcap does not support symbolic links, so a potential symbolic link has to be resolved first.
resolved_symlink=$(readlink -f ${9})

# Setcap using password.
echo ${10} | sudo -S setcap cap_net_raw+ep ${resolved_symlink}

# Update the links and cache to the shared catkin libraries.
# See https://stackoverflow.com/questions/9843178/linux-capabilities-setcap-seems-to-disable-ld-library-path
sudo ldconfig /opt/ros/$ROS_DISTRO/lib

# Launch the node.
roslaunch anydrive_ethercat_ros anydrive_ethercat_ros.launch output:="${1}" launch_prefix:="${2}" setup_name:="${3}" setup_file:="${4}" time_step:="${5}" ros_prefix:="${6}" ros_config_file:="${7}" run_gui:="${8}"

