/*
 * XsensmtTriggeringCosmoNode.cpp
 *
 *  Created on: Sep 24, 2018
 *      Author: jelavice
 */

#include <xsensmt_cosmo/XsensmtTriggeringCosmoNode.hpp>
#include <any_node/Node.hpp>

namespace xsensmt {

XsensmtTriggeringCosmoNode::XsensmtTriggeringCosmoNode(
		any_node::Node::NodeHandlePtr nh) :
		Base(nh) {

}

bool XsensmtTriggeringCosmoNode::advertiseTopic() {

	cosmo_ros::PublisherRosOptionsPtr options = std::make_shared
			< cosmo_ros::PublisherRosOptions > ("imu", getNodeHandle());
	options->rosQueueSize_ = 10u;
	options->rosLatch_ = false;
	options->autoPublishRos_ = false;

	imuPublisher_ = cosmo_ros::advertiseShmRos<ImuShm, ImuRos,
			any_measurements_ros::ConversionTraits>("imu", options);

	if (publishRos_) {
		any_worker::WorkerOptions pubOptions;
		pubOptions.callback_ = std::bind(
				&XsensmtTriggeringCosmoNode::publishWorker, this,
				std::placeholders::_1);
		pubOptions.defaultPriority_ = 9; // this has low priority
		pubOptions.name_ = "XsensmtTriggeringCosmoNode::publisherWorker";
		pubOptions.timeStep_ = std::numeric_limits<double>::infinity();
		this->addWorker(pubOptions);
	}

	return addWorker(ros::this_node::getName() + "::updateWorker",
			param<double>("time_step", 1.0),
			&XsensmtTriggeringCosmoNode::update, this, 98);;

}

bool XsensmtTriggeringCosmoNode::update(const any_worker::WorkerEvent& event) {

	/* publish sensor data */
	ImuMeasurement data;
	static int err = 0;
	static int prev_err = 0;

	/* check error code */
	err = xsensMTSensor_.getStatus().errCode_;
	if (err != prev_err) {
		/* err status changed */
		if (err != 0) {
			ROS_WARN("XsensmtCosmoNode: errCode_=%d, %s ", err,
					xsensMTSensor_.errToString(err).c_str());
		} else {
			ROS_INFO("XsensmtCosmoNode: sensor connected & running");
			ROS_INFO("XsensmtCosmoNode: at port: %s",
					xsensMTSensor_.getConfigParam().serialDev_.c_str());
		}
	}
	prev_err = err;

	/* try to extract & publish sensor data */
	if (xsensMTSensor_.getMeasurement(data)) {
		//    imuMsg.header.seq = data.counter_;
		//shmMessage_.time_= data.timestamp_;
		shmMessage_.time_ = any_measurements_ros::fromRos(ros::Time::now());
		shmMessage_.angularVelocity_.x() = data.angularVelocity_[0];
		shmMessage_.angularVelocity_.y() = data.angularVelocity_[1];
		shmMessage_.angularVelocity_.z() = data.angularVelocity_[2];
		shmMessage_.linearAcceleration_.x() = data.linearAcceleration_[0];
		shmMessage_.linearAcceleration_.y() = data.linearAcceleration_[1];
		shmMessage_.linearAcceleration_.z() = data.linearAcceleration_[2];
		shmMessage_.triggerIndicator_ = data.triggerIndicator_;

		any_measurements_ros::toRos(shmMessage_, rosMessage_);
		imuPublisher_->publish(shmMessage_, rosMessage_,
				std::chrono::microseconds { 200 });
	}

	// Notify the publish worker
	updateCounter_++;
	cvUpdate_.notify_all();

	return true;

}

void XsensmtTriggeringCosmoNode::getParameters(){

	getBasicParameters();

	/* synchronization parameters*/
	getNodeHandle().param<bool>("sensor/send_sync_out_signal",
	configParam_.sendSync_, false);

	getNodeHandle().param<int>("synchronization/skip_factor",
			configParam_.skipFactor_, 9);

	getNodeHandle().param<int>("synchronization/pulse_width",
			configParam_.pulseWidth_, 1000);
}

} /* namespace */

