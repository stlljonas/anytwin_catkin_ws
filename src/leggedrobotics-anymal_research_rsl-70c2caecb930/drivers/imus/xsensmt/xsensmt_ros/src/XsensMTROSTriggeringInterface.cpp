/*
 * XsensMTROSTriggeringInterface.cpp
 *
 *  Created on: Sep 13, 2018
 *      Author: jelavice
 */

#include "xsensmt_ros/XsensMTROSTriggeringInterface.h"
#include "any_msgs/ImuWithTrigger.h"


namespace xsensmt {

//TODO maybe stick this inside some conversion traits or sth, but then I'll break a lot of code
/// called on every update step of the node
bool XsensMTROSTriggeringInterface::update(
		const any_worker::WorkerEvent& event) {
	/* publish sensor data */
	ImuMeasurement data;
	static int err = 0;
	static int prev_err = 0;

	/* check error code */
	err = xsensMTSensor_.getStatus().errCode_;
	if (err != prev_err) {
		/* err status changed */
		if (err != 0) {
			ROS_WARN("XsensMTROSInterface: errCode_=%d, %s ", err,
					xsensMTSensor_.errToString(err).c_str());
		} else {
			ROS_INFO("XsensMTROSInterface: sensor connected & running");
			ROS_INFO("XsensMTROSInterface: at port: %s",
					xsensMTSensor_.getConfigParam().serialDev_.c_str());
		}
	}
	prev_err = err;

	/* try to extract & publish sensor data */
	if (xsensMTSensor_.getMeasurement(data)) {
		any_msgs::ImuWithTrigger imuMsg;

		imuMsg.imu.header.seq = data.counter_;
		imuMsg.imu.header.frame_id = frameId_;
		imuMsg.imu.header.stamp = ros::Time().fromNSec(
				std::chrono::duration_cast < std::chrono::nanoseconds
						> (data.timestamp_.time_since_epoch()).count());

		imuMsg.imu.angular_velocity.x = data.angularVelocity_[0];
		imuMsg.imu.angular_velocity.y = data.angularVelocity_[1];
		imuMsg.imu.angular_velocity.z = data.angularVelocity_[2];
		imuMsg.imu.linear_acceleration.x = data.linearAcceleration_[0];
		imuMsg.imu.linear_acceleration.y = data.linearAcceleration_[1];
		imuMsg.imu.linear_acceleration.z = data.linearAcceleration_[2];
		imuMsg.triggerIndicator = data.triggerIndicator_;
		if (imuPublisher_.getNumSubscribers() > 0) {
			imuPublisher_.publish(imuMsg);
		}
	}

	return true;
}

void XsensMTROSTriggeringInterface::advertiseTopic() {
	imuPublisher_ = getNodeHandle().advertise < any_msgs::ImuWithTrigger
			> ("imu", ROSQueueDepth_);

}

void XsensMTROSTriggeringInterface::getParameters(){

	getBasicParameters();
	/* synchronization parameters*/
	configParam_.sendSync_ = param<bool>("sensor/send_sync_out_signal", false);

	configParam_.receiveSync_ = param<bool>("sensor/receive_sync_in_signal", false);

	configParam_.skipFactor_ = param<int>("synchronization/skip_factor", 9);

	configParam_.pulseWidth_ = param<int>("synchronization/pulse_width", 1000);

}



} /* namespace */

