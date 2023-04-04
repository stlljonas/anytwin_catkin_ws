/*
 * FootCalibrator.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: Dario Bellicoso
 */

#include "loco_ros_anymal/foot_calibrator/FootCalibrator.hpp"

namespace loco_ros_anymal {


FootCalibrator::FootCalibrator()
{

}


FootCalibrator::~FootCalibrator()
{

}


void FootCalibrator::initialize() {
  for (auto& service: calibrateFootClient_) {
    service.second.shutdown();
  }

  initializeServices();
}


void FootCalibrator::initializeServices() {

  //--- start a worker to calibrate the lf foot
  roco::WorkerOptions calibrateFootLfWorkerOptions;
  calibrateFootLfWorkerOptions.autostart_ = false;
  calibrateFootLfWorkerOptions.frequency_ = 0.0;
  calibrateFootLfWorkerOptions.name_ = "dynamic_gaits_ros_calibrate_foot_lf";
  calibrateFootLfWorkerOptions.priority_ = 0;
  calibrateFootLfWorkerOptions.synchronous_ = false;
  calibrateFootLfWorkerOptions.callback_ = boost::bind(&FootCalibrator::calibrateFootWorkerLf, this, _1);
  calibrateFootWorkerHandles_[anymal_model::LimbEnum::LF_LEG] = addWorker(calibrateFootLfWorkerOptions);
  //---

  //--- start a worker to calibrate the rf foot
  roco::WorkerOptions calibrateFootRfWorkerOptions;
  calibrateFootRfWorkerOptions.autostart_ = false;
  calibrateFootRfWorkerOptions.frequency_ = 0.0;
  calibrateFootRfWorkerOptions.name_ = "dynamic_gaits_ros_calibrate_foot_rf";
  calibrateFootRfWorkerOptions.priority_ = 0;
  calibrateFootRfWorkerOptions.synchronous_ = false;
  calibrateFootRfWorkerOptions.callback_ = boost::bind(&FootCalibrator::calibrateFootWorkerRf, this, _1);
  calibrateFootWorkerHandles_[anymal_model::LimbEnum::RF_LEG] = addWorker(calibrateFootRfWorkerOptions);
  //---

  //--- start a worker to calibrate the lh foot
  roco::WorkerOptions calibrateFootLhWorkerOptions;
  calibrateFootLhWorkerOptions.autostart_ = false;
  calibrateFootLhWorkerOptions.frequency_ = 0.0;
  calibrateFootLhWorkerOptions.name_ = "dynamic_gaits_ros_calibrate_foot_lh";
  calibrateFootLhWorkerOptions.priority_ = 0;
  calibrateFootLhWorkerOptions.synchronous_ = false;
  calibrateFootLhWorkerOptions.callback_ = boost::bind(&FootCalibrator::calibrateFootWorkerLh, this, _1);
  calibrateFootWorkerHandles_[anymal_model::LimbEnum::LH_LEG] = addWorker(calibrateFootLhWorkerOptions);
  //---

  //--- start a worker to calibrate the rh foot
  roco::WorkerOptions calibrateFootRhWorkerOptions;
  calibrateFootRhWorkerOptions.autostart_ = false;
  calibrateFootRhWorkerOptions.frequency_ = 0.0;
  calibrateFootRhWorkerOptions.name_ = "dynamic_gaits_ros_calibrate_foot_rh";
  calibrateFootRhWorkerOptions.priority_ = 0;
  calibrateFootRhWorkerOptions.synchronous_ = false;
  calibrateFootRhWorkerOptions.callback_ = boost::bind(&FootCalibrator::calibrateFootWorkerRh, this, _1);
  calibrateFootWorkerHandles_[anymal_model::LimbEnum::RH_LEG] = addWorker(calibrateFootRhWorkerOptions);
  //---

}

bool FootCalibrator::calibrateFootWorkerLf(const roco::WorkerEvent& event) {
  ROS_INFO("[FootCalibrator::calibrateFootWorkerLf] Resetting LF foot sensor.");
  bool result = true;
  any_msgs::SetUInt32 calibrateFootService;
  calibrateFootService.request.data = calibrateUsingNumSamples_;
  if(!calibrateFootClientLf_.call(calibrateFootService)) {
    ROS_WARN("[FootCalibrator::calibrateFootWorkerLf] Resetting LF foot sensor returned false!");
    result = false;
  }
  return result;
}

bool FootCalibrator::calibrateFootWorkerRf(const roco::WorkerEvent& event) {
  ROS_INFO("[FootCalibrator::calibrateFootWorkerRf] Resetting RF foot sensor.");
  bool result = true;
  any_msgs::SetUInt32 calibrateFootService;
  calibrateFootService.request.data = calibrateUsingNumSamples_;
  if(!calibrateFootClientRf_.call(calibrateFootService)) {
    ROS_WARN("[FootCalibrator::calibrateFootWorkerRf] Resetting RF foot sensor returned false!");
    result = false;
  }
  return result;
}

bool FootCalibrator::calibrateFootWorkerLh(const roco::WorkerEvent& event) {
  ROS_INFO("[FootCalibrator::calibrateFootWorkerLh] Resetting LH foot sensor.");
  bool result = true;
  any_msgs::SetUInt32 calibrateFootService;
  calibrateFootService.request.data = calibrateUsingNumSamples_;
  if(!calibrateFootClientLh_.call(calibrateFootService)) {
    ROS_WARN("[FootCalibrator::calibrateFootWorkerLh] Resetting LH foot sensor returned false!");
    result = false;
  }
  return result;
}

bool FootCalibrator::calibrateFootWorkerRh(const roco::WorkerEvent& event) {
  ROS_INFO("[FootCalibrator::calibrateFootWorkerRh] Resetting RH foot sensor.");
  bool result = true;
  any_msgs::SetUInt32 calibrateFootService;
  calibrateFootService.request.data = calibrateUsingNumSamples_;
  if(!calibrateFootClientRh_.call(calibrateFootService)) {
    ROS_WARN("[FootCalibrator::calibrateFootWorkerRh] Resetting RH foot sensor returned false!");
    result = false;
  }
  return result;
}


} /* namespace loco_ros_anymal */
