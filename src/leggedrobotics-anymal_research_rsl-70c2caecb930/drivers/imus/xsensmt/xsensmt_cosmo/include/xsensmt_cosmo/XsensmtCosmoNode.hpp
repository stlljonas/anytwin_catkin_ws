/** \file: XsensmtCosmoNodeNode.h
    \brief: ROS interface node for driver configuration & diagnosing
  */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <cstdint>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <xsensmt_interface/ImuMeasurement.h>
#include <xsensmt_interface/ImuConfig.h>
#include <xsensmt_interface/ImuStatus.h>
#include <xsensmt_interface/XsensMTInterface.h>

#include <any_node/Node.hpp>
#include <message_logger/message_logger.hpp>

#include "any_measurements_ros/any_measurements_ros.hpp"
#include "cosmo_ros/cosmo_ros.hpp"

using namespace any_node;

namespace xsensmt {

class XsensmtCosmoNode: public any_node::Node {
public:
  using ImuShm = any_measurements::Imu;
  using ImuRos = sensor_msgs::Imu;
  using ImuPublisherPtr = cosmo_ros::PublisherRosPtr<ImuShm, ImuRos, any_measurements_ros::ConversionTraits>;

  /* constructor needs to take a shared_ptr to a ros::Nodehandle instance. */
  XsensmtCosmoNode() = delete;
  XsensmtCosmoNode(any_node::Node::NodeHandlePtr nh);

  /* destructor */
  ~XsensmtCosmoNode() override = default;

  /// initialize node
  bool init() override;
  /// safely stop node and quit
  void cleanup() override;
  /// shut down publish worker here
  void preCleanup() override;
  /// called on every update step of the node
  virtual bool update(const any_worker::WorkerEvent& event);

  bool publishWorker(const any_worker::WorkerEvent& workerEvent);

protected:
  /// get parameters from ROS param server
  virtual void getParameters();

  void getBasicParameters();

  /// xsensMTInterface object
  XsensMTInterface xsensMTSensor_;

  ImuConfig configParam_;
  std::string frameId_;
  ros::Timer timer_;

  bool publishRos_;
  boost::condition_variable_any cvUpdate_;
  boost::mutex mutexPublishUpdate_;
  boost::atomic<bool> stopUpdating_;
  std::atomic<unsigned long> updateCounter_;

private:

  virtual bool advertiseTopic();
  ImuPublisherPtr imuPublisher_;
  ImuShm shmMessage_;
  ImuRos rosMessage_;

};

} // namespace


