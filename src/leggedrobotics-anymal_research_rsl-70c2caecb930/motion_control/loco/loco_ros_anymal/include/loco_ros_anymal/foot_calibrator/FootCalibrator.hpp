/*
 * FootCalibrator.hpp
 *
 *  Created on: Apr 3, 2016
 *      Author: Dario Bellicoso
 */

#pragma once


// worker interface
#include <roco/workers/WorkerEvent.hpp>
#include <roco/workers/WorkerHandle.hpp>

// ros
#include <ros/ros.h>
#include <any_msgs/SetUInt32.h>

// anymal model
#include <anymal_model/common/enums.hpp>


namespace loco_ros_anymal {

class FootCalibrator
{
 public:
  FootCalibrator();
  virtual ~FootCalibrator();

  void initialize();

  bool calibrateFootWorkerLf(const roco::WorkerEvent& event);
  bool calibrateFootWorkerRf(const roco::WorkerEvent& event);
  bool calibrateFootWorkerLh(const roco::WorkerEvent& event);
  bool calibrateFootWorkerRh(const roco::WorkerEvent& event);

 protected:
  void initializeServices();

 private:
  std::map<anymal_model::LimbEnum, roco::WorkerHandle> calibrateFootWorkerHandles_;
  std::map<anymal_model::LimbEnum, bool> didStartCalibrationServiceOnceSinceSwing_;
  std::map<anymal_model::LimbEnum, ros::ServiceClient> calibrateFootClient_;
};

} /* namespace loco_ros_anymal */
