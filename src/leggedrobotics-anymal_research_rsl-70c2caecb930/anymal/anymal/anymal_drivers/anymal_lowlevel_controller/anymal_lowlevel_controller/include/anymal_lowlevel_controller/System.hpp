#pragma once


// c++
#include <atomic>
#include <memory>
#include <mutex>

#include <ros/ros.h>

// anydrive ethercat ros
#include <anydrive_ethercat_ros/AnydriveManagerEthercatRos.hpp>
#include <anymal_description/AnymalDescription.hpp>

// anymal lowlevel controller
#include "anymal_lowlevel_controller/state_machine/substates.hpp"

namespace anymal_lowlevel_controller {


class System
{
public:
  using AD = anymal_description::AnymalDescription;
  using AnydriveManagerPtr = anydrive_ethercat_ros::AnydriveManagerEthercatRosPtr;

protected:
  AnydriveManagerPtr anydriveManager_;
  std_utils::EnumArray<AD::ActuatorEnum, anydrive::AnydrivePtr> anydrives_; // Faster anydrives access.

  double timeStep_;
  ros::NodeHandle nodeHandle_;

  std::atomic<bool> isShutdownRequested_{false};
  std::atomic<bool> isActuatorSetup_{false};
  std::atomic<bool> isActuatorCommunicationEnabled_{true};

  std::mutex anydriveMutex_;

 public:
  System() = default;
  ~System() = default;

  bool init(ros::NodeHandle& nh, const double timeStep);
  bool initActuators();
  bool startup();
  void preCleanup();
  void cleanup();

  void enableActuatorCommunication(bool enable);
  bool isActutatorCommunicationEnabled();
  bool isActuatorSetup();
  void updateSensors(const any_worker::WorkerEvent& event);
  const AnydriveManagerPtr& getAnydriveManager();
  const anydrive::AnydrivePtr& getAnydrive(AD::ActuatorEnum actuatorEnum);
  state_machine::Substates getActiveSubstates();

  state_machine::AnydrivesStateEnum getAnydrivesStateEnum();
};

using SystemPtr = std::shared_ptr<System>;


} // anymal_lowlevel_controller
