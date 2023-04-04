/*!
* @file     JoyStateEstimation.hpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/

#pragma once

#include <string>

#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <joy_manager/ModuleBase.hpp>




namespace joy_manager {

class JoyStateEstimation : public joy_manager::ModuleBase
{
 public:
  JoyStateEstimation();
  virtual ~JoyStateEstimation();
  void init(const ros::NodeHandle& nh, joy_manager::JoyManager* joyManager, const std::string& name);
  void cleanup();
  void processCommand(const std::string& command, std::string* answer);

 protected:
  void resetStateEstimator(std::string* answer);
  void resetStateEstimatorHere(std::string* answer);
  void calibrateForceSensors(std::string* answer);
  void runCalibrationThread();
  void runTimerThread();

  ros::ServiceClient resetStateEstimatorClient_;
  ros::ServiceClient resetStateEstimatorHereClient_;
  ros::ServiceClient calibrateForceSencorsClient_;

  const double kTimer_ = 3.0;
  boost::thread optoforceThread_;
  boost::thread timerThread_;
  boost::atomic<bool> isCalibrating_;
  boost::atomic<bool> isTiming_;
  boost::atomic<bool> hasCalibrationFinished_;
  boost::atomic<bool> hasCalibrationFailed_;
  boost::atomic<bool> hasTimerFinished_;
};

} // namespace joy_manager
