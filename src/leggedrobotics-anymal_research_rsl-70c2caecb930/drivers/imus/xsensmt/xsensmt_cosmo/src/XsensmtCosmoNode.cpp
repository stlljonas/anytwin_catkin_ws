/** \file: XsensmtCosmoNodeNode.cpp
    \brief: ROS interface node for driver configuration & diagnosing
  */

#include "xsensmt_cosmo/XsensmtCosmoNode.hpp"
#include <chrono>

using namespace any_node;

namespace xsensmt {

XsensmtCosmoNode::XsensmtCosmoNode(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh), publishRos_(false), stopUpdating_(false) {
  updateCounter_ = 01u;
}

/// initialize node
bool XsensmtCosmoNode::init() {
  /* read in parameters from ROS param server */
  getParameters();

  /** try to connect to sensor for the first time
   *  only needed for the first time
   *  after this, the XsensMTInterface class will automatically
   *  try to reconnect when sensor is lost.
   */
  while (getNodeHandle().ok()) {
    /* configure sensor interface */
    if (xsensMTSensor_.init()) {
      ROS_INFO("XsensmtCosmoNode: driver initialization successful");
    } else {
      ROS_WARN("XsensmtCosmoNode: driver initialization failed, retry in 0.5s");
      usleep(500000);
      continue;
    }

    if (xsensMTSensor_.configure(configParam_)) {
      ROS_INFO("XsensmtCosmoNode: sensor configuration successful");
      ROS_INFO("XsensmtCosmoNode: sensor connected & running");
      ROS_INFO("XsensmtCosmoNode: at port: %s", xsensMTSensor_.getConfigParam().serialDev_.c_str());

      // generate some reports
      if (configParam_.correctTiming_) {
        ROS_INFO("XsensmtCosmoNode: data timing correction enabled");
      } else {
        ROS_INFO("XsensmtROSQueueDepth_CosmoNode: data timing correction disabled");
      }

      break;
    } else {
      int err = xsensMTSensor_.getStatus().errCode_;
      ROS_WARN("XsensmtCosmoNode: sensor configuration failed: %s, retry in 0.5s", xsensMTSensor_.errToString(err).c_str());
      usleep(500000);
      continue;
    }
  }

  rosMessage_.header.frame_id = frameId_;

  return advertiseTopic();
}

/// shut down publish worker here
void XsensmtCosmoNode::preCleanup() {
  stopUpdating_ = true;
  cvUpdate_.notify_all();
}

/// safely stop node and quit
void XsensmtCosmoNode::cleanup() { xsensMTSensor_.cleanup(); }

/// called on every update step of the node
bool XsensmtCosmoNode::update(const any_worker::WorkerEvent& event) {
  /* publish sensor data */
  ImuMeasurement data;
  static int err = 0;
  static int prev_err = 0;

  /* check error code */
  err = xsensMTSensor_.getStatus().errCode_;
  if (err != prev_err) {
    /* err status changed */
    if (err != 0) {
      ROS_WARN("XsensmtCosmoNode: errCode_=%d, %s ", err, xsensMTSensor_.errToString(err).c_str());
    } else {
      ROS_INFO("XsensmtCosmoNode: sensor connected & running");
      ROS_INFO("XsensmtCosmoNode: at port: %s", xsensMTSensor_.getConfigParam().serialDev_.c_str());
    }
  }
  prev_err = err;

  /* try to extract & publish sensor data */
  if (xsensMTSensor_.getMeasurement(data)) {
    //    imuMsg.header.seq = data.counter_;
    // shmMessage_.time_= data.timestamp_;
    shmMessage_.time_ = any_measurements_ros::fromRos(ros::Time::now());
    shmMessage_.angularVelocity_.x() = data.angularVelocity_[0];
    shmMessage_.angularVelocity_.y() = data.angularVelocity_[1];
    shmMessage_.angularVelocity_.z() = data.angularVelocity_[2];
    shmMessage_.linearAcceleration_.x() = data.linearAcceleration_[0];
    shmMessage_.linearAcceleration_.y() = data.linearAcceleration_[1];
    shmMessage_.linearAcceleration_.z() = data.linearAcceleration_[2];
    any_measurements_ros::toRos(shmMessage_, rosMessage_);
    imuPublisher_->publish(shmMessage_, rosMessage_, std::chrono::microseconds{200});
  }

  // Notify the publish worker
  updateCounter_++;
  cvUpdate_.notify_all();

  return true;
}

bool XsensmtCosmoNode::publishWorker(const any_worker::WorkerEvent& workerEvent) {
  unsigned long localCounter = 0lu;

  while (!stopUpdating_) {
    boost::unique_lock<boost::mutex> lock{mutexPublishUpdate_};
    cvUpdate_.wait(lock, [this, localCounter]() {
      if (stopUpdating_) return true;
      return (updateCounter_ > localCounter);
    });
    localCounter = updateCounter_;

    // Stop immediately
    if (stopUpdating_) {
      return true;
    }

    imuPublisher_->sendRos();
  }

  return true;
}

bool XsensmtCosmoNode::advertiseTopic() {
  cosmo_ros::PublisherRosOptionsPtr options = std::make_shared<cosmo_ros::PublisherRosOptions>("imu", getNodeHandle());
  options->rosQueueSize_ = 10u;
  options->rosLatch_ = false;
  options->autoPublishRos_ = false;

  imuPublisher_ = cosmo_ros::advertiseShmRos<ImuShm, ImuRos, any_measurements_ros::ConversionTraits>("imu", options);

  if (publishRos_) {
    any_worker::WorkerOptions pubOptions;
    pubOptions.callback_ = std::bind(&XsensmtCosmoNode::publishWorker, this, std::placeholders::_1);
    pubOptions.defaultPriority_ = 9;  // this has low priority
    pubOptions.name_ = "XsensmtCosmoNode::publisherWorker";
    pubOptions.timeStep_ = std::numeric_limits<double>::infinity();
    this->addWorker(pubOptions);
  }

  return addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0), &XsensmtCosmoNode::update, this, 98);
}

void XsensmtCosmoNode::getParameters() { getBasicParameters(); }

void XsensmtCosmoNode::getBasicParameters() {
  publishRos_ = param<bool>("publish_ros", false);

  frameId_ = param<std::string>("ros/frame_id", "/xsensmt_link");

  configParam_.bufferSize_ = param<int>("sensor/queue_capacity", 3);

  configParam_.serialDev_ = param<std::string>("sensor/serial_device", "/dev/ttyxsens");

  configParam_.serialBaudrate_ = param<int>("sensor/serial_baudrate", 921600);

  configParam_.serialKey_ = param<std::string>("sensor/serial_key", "R945-E2EF-RPP0-W86J-P408");

  configParam_.outputFrequency_ = param<int>("sensor/output_frequency", 400);

  configParam_.outputOrientation_ = param<bool>("sensor/output_orientation", false);

  configParam_.correctTiming_ = param<bool>("sensor/correct_timing", true);

  configParam_.retryInterval_ = param<int>("sensor/retry_interval", 500);

  configParam_.maxInterPacketTime_ = param<int>("sensor/max_inter_packet_time", 500);

  configParam_.sendStatusWord_ = param<bool>("sensor/send_status_word", true);

  /* synchronization parameters*/
  configParam_.sendSync_ = param<bool>("sensor/send_sync_out_signal", false);

  configParam_.receiveSync_ = param<bool>("sensor/receive_sync_in_signal", false);

  configParam_.skipFactor_ = param<int>("synchronization/skip_factor", 9);

  configParam_.pulseWidth_ = param<int>("synchronization/pulse_width", 1000);
}

}  // namespace xsensmt
