/** \file: XsensMTROSInterfaceNode.cpp
    \brief: ROS interface node for driver configuration & diagnosing
  */

#include <xsensmt_ros/XsensMTROSInterface.h>

using namespace any_node;

namespace xsensmt {

/// initialize node
bool XsensMTROSInterface::init() {
  /* read in parameters from ROS param server */
  getParameters();

  /* initialize ROS publisher */
  advertiseTopic();

  /** try to connect to sensor for the first time
   *  only needed for the first time
   *  after this, the XsensMTInterface class will automatically
   *  try to reconnect when sensor is lost.
   */
  while (getNodeHandle().ok()) {
    /* configure sensor interface */
    if (xsensMTSensor_.init()) {
      ROS_INFO("XsensMTROSInterface: driver initialization successful");
    } else {
      ROS_WARN("XsensMTROSInterface: driver initialization failed, retry in 0.5s");
      usleep(500000);
      continue;
    }

    if (xsensMTSensor_.configure(configParam_)) {
      ROS_INFO("XsensMTROSInterface: sensor configuration successful");
      ROS_INFO("XsensMTROSInterface: sensor connected & running");
      ROS_INFO("XsensMTROSInterface: at port: %s", xsensMTSensor_.getConfigParam().serialDev_.c_str());

      // generate some reports
      if (configParam_.correctTiming_) {
        ROS_INFO("XsensMTROSInterface: data timing correction enabled");
      } else {
        ROS_INFO("XsensMTROSInterface: data timing correction disabled");
      }

      break;
    } else {
      int err = xsensMTSensor_.getStatus().errCode_;
      ROS_WARN("XsensMTROSInterface: sensor configuration failed: %s, retry in 0.5s", xsensMTSensor_.errToString(err).c_str());
      usleep(500000);
      continue;
    }
  }
  return addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0), &XsensMTROSInterface::update, this, 10);
}

/// safely stop node and quit
void XsensMTROSInterface::cleanup() { xsensMTSensor_.cleanup(); }

/// called on every update step of the node
bool XsensMTROSInterface::update(const any_worker::WorkerEvent& event) {
  /* publish sensor data */
  ImuMeasurement data;
  static int err = 0;
  static int prev_err = 0;

  /* check error code */
  err = xsensMTSensor_.getStatus().errCode_;
  if (err != prev_err) {
    /* err status changed */
    if (err != 0) {
      ROS_WARN("XsensMTROSInterface: errCode_=%d, %s ", err, xsensMTSensor_.errToString(err).c_str());
    } else {
      ROS_INFO("XsensMTROSInterface: sensor connected & running");
      ROS_INFO("XsensMTROSInterface: at port: %s", xsensMTSensor_.getConfigParam().serialDev_.c_str());
    }
  }
  prev_err = err;

  /* try to extract & publish sensor data */
  if (xsensMTSensor_.getMeasurement(data)) {
    sensor_msgs::Imu imuMsg;
    any_msgs::SensorTimeInfo imuTimeMsg;

    imuMsg.header.seq = data.counter_;
    imuMsg.header.frame_id = frameId_;
    if (useRosTime_) {
      imuMsg.header.stamp = ros::Time().now();
    } else {
      imuMsg.header.stamp =
          ros::Time().fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(data.timestamp_.time_since_epoch()).count());
    }

    imuTimeMsg.header.stamp = imuMsg.header.stamp;
    imuTimeMsg.counter = data.counter_;  // Counter from Imu

    imuMsg.angular_velocity.x = data.angularVelocity_[0];
    imuMsg.angular_velocity.y = data.angularVelocity_[1];
    imuMsg.angular_velocity.z = data.angularVelocity_[2];
    imuMsg.linear_acceleration.x = data.linearAcceleration_[0];
    imuMsg.linear_acceleration.y = data.linearAcceleration_[1];
    imuMsg.linear_acceleration.z = data.linearAcceleration_[2];
    if (imuPublisher_.getNumSubscribers() > 0) {
      imuPublisher_.publish(imuMsg);
      imuCounterPublisher_.publish(imuTimeMsg);
    }
  }

  return true;
}

/// get parameters from ROS param server
void XsensMTROSInterface::getParameters() { getBasicParameters(); }

void XsensMTROSInterface::getBasicParameters() {
  frameId_ = param<std::string>("ros/frame_id", "xsensmt_link");

  ROSQueueDepth_ = param<int>("ros/queue_depth", 100);

  useRosTime_ = param<int>("ros/use_ros_time", false);

  configParam_.bufferSize_ = param<int>("sensor/queue_capacity", 3);

  configParam_.serialBaudrate_ = param<int>("sensor/serial_baudrate", 921600);

  configParam_.serialKey_ = param<std::string>("sensor/serial_key", "R945-E2EF-RPP0-W86J-P408");

  configParam_.outputFrequency_ = param<int>("sensor/output_frequency", 400);

  configParam_.outputOrientation_ = param<bool>("sensor/output_orientation", false);

  configParam_.correctTiming_ = param<bool>("sensor/correct_timing", true);

  configParam_.serialDev_ = param<std::string>("sensor/serial_device", "/dev/ttyxsens");

  configParam_.retryInterval_ = param<int>("sensor/retry_interval", 500);

  configParam_.maxInterPacketTime_ = param<int>("sensor/max_inter_packet_time", 500);

  configParam_.sendStatusWord_ = param<bool>("sensor/send_status_word", true);

  /* synchronization parameters*/
  configParam_.sendSync_ = param<bool>("sensor/send_sync_out_signal", false);

  configParam_.receiveSync_ = param<bool>("sensor/receive_sync_in_signal", false);

  configParam_.skipFactor_ = param<int>("synchronization/skip_factor", 9);

  configParam_.pulseWidth_ = param<int>("synchronization/pulse_width", 1000);
}

void XsensMTROSInterface::advertiseTopic() {
  imuPublisher_ = getNodeHandle().advertise<sensor_msgs::Imu>("imu", ROSQueueDepth_);
  imuCounterPublisher_ = getNodeHandle().advertise<any_msgs::SensorTimeInfo>("imu/time_info", ROSQueueDepth_);
}

}  // namespace xsensmt
