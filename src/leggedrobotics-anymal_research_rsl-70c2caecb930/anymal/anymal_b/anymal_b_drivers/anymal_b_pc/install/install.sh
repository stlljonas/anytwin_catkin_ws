#!/usr/bin/env bash

set -e

if [[ $# -ne 2 ]]; then
    echo "Usage: install.sh <robot-name> <pc-type>"
    exit -1
fi

export robot_name=$1
export pc_type=$2

export dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# check if anymal-pc package is installed
if $(dpkg -s anymal-pc > /dev/null 2>&1); then
    echo "anymal-pc package is installed, which handles the installation of system configuration files. Nothing to do."
    exit 0
fi

# create ros.conf if it does not exist
if [[ ! -f /etc/robot/ros.conf ]]; then
    echo "Did not find /etc/robot/ros.conf! Creating default file."
    mkdir -p /etc/robot
    envsubst < ${dir}/etc/robot/ros.conf.template > /etc/robot/ros.conf
fi

source ${dir}/utils.bash
source /etc/robot/ros.conf

# do some sanity checks on the provided arguments
if [[ ${robot_name} != ${ANYMAL_NAME} ]]; then
    echo "Name of the robot passed to the script and found in /etc/robot/ros.conf do not match! Aborting."
    exit -1
fi

if [[ ${pc_type} != ${ANYMAL_PC} ]]; then
    echo "PC type passed to the script and found in /etc/robot/ros.conf do not match! Aborting."
    exit -1
fi


## install systemd files
if [[ ${ANYMAL_PC} == "lpc" ]]; then
  update_systemd_service ${dir} anymal-roscore.service
  update_systemd_service ${dir} anymal-load-config.service
fi

update_systemd_service_template ${dir} anymal-sw-stack@.service ${ANYMAL_PC} ${ANYMAL_PC}_background

# remove deprecated services
remove_systemd_service anymal-rpsm.service # removed 17.03.2020
remove_systemd_service anymal-hri.service # removed 17.03.2020
remove_systemd_service anymal-logger.service # removed 18.03.2020
