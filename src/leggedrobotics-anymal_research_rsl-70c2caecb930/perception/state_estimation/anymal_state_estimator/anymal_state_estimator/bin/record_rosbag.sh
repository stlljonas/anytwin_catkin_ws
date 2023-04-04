#! /bin/bash
#record topics necessary to run the tsif standalone

outpath="$1"
now="$(date +"%F-%H-%M-%S")"

# List of useful colors
COLOR_RESET="\033[0m"
COLOR_WARN="\033[0;33m"

# Message for the user
msg_to_user="${COLOR_WARN}[Reminder] Did you remember to set 'publish_imu' to 'true' in 'anymal_X_lpc_basic/anymal_drivers.launch'?${COLOR_RESET}"

if [ "$outpath" == "" ]; then
  outpath="."
fi

echo -e "${msg_to_user}"

rosparam dump $outpath/$now.yaml

rosbag record --output-name=$outpath/$now \
/lidar/point_cloud_filtered
/sensors/imu \
/sensors/contact_force_lf_foot \
/sensors/contact_force_lh_foot \
/sensors/contact_force_rf_foot \
/sensors/contact_force_rh_foot \
/anymal_lowlevel_controller/actuator_readings \
/anymal_highlevel_controller/force_calibrator_commands \
/state_estimator/pose_in_odom \
/state_estimator/twist \
/state_estimator/anymal_state \
/state_estimator/log/imuAngularVelocityBias \
/state_estimator/log/imuAngularVelocityBiasCovarianceDiagonal \
/state_estimator/log/imuLinearAccelerationBias \
/state_estimator/log/imuLinearAccelerationBiasCovarianceDiagonal \
/state_estimator/log/pitchBaseToWorldInDeg \
/state_estimator/log/pitchMapToOdomInDeg \
/state_estimator/log/rollBaseToWorldInDeg \
/state_estimator/log/rollMapToOdomInDeg \
/state_estimator/log/yawBaseToWorldInDeg \
/state_estimator/log/yawMapToOdomInDeg \

echo -e "${msg_to_user}"
