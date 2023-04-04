// std
#include <csignal>

// anydrive
#include <anydrive/Exception.hpp>
#include <anydrive/fsm/StateEnum.hpp>
#include <anydrive_ethercat/setup/SetupEthercat.hpp>
#include <anydrive_ethercat_ros/conversions.hpp>

// anymal lowlevel controller
#include "anymal_lowlevel_controller/System.hpp"

namespace anymal_lowlevel_controller {

bool System::init(ros::NodeHandle& nh, const double timeStep)
{
  nodeHandle_ = nh;
  timeStep_ = timeStep;
  return true;
}

bool System::initActuators() {
  // Actuators.
  const bool standalone = false;
  const bool installSignalHandler = false;
  anydriveManager_.reset(new anydrive_ethercat_ros::AnydriveManagerEthercatRos(
      standalone, installSignalHandler, timeStep_, nodeHandle_, param_io::param<std::string>(nodeHandle_, "ros_prefix", "")));

  // Read ANYdrive setup from Ros parameter server.
  XmlRpc::XmlRpcValue params;
  try {
    if (!param_io::getParam(nodeHandle_, "anydrive_setup", params)) {
      MELO_ERROR_STREAM("Container 'anydrive_setup' does not exist.")
      return false;
    }
  } catch (const XmlRpc::XmlRpcException& exception) {
    MELO_ERROR_STREAM("Caught an XmlRpc exception while getting container 'anydrive_setup': " << exception.getMessage() << ".")
    return false;
  }
  auto setupPtr = anydrive::setup::SetupPtr(new anydrive_ethercat::setup::SetupEthercat());
  try {
    if (!anydrive_ethercat_ros::readSetupParameters(params, setupPtr)) {
      MELO_ERROR_STREAM("Could not read ANYdrive setup parameters.")
      return false;
    }
  } catch (anydrive::Exception& exception) {
    MELO_ERROR_STREAM("Caught an ANYdrive exception: " << exception.what())
    return false;
  }

  // Load setup.
  if (!anydriveManager_->loadSetup(setupPtr)) {
    MELO_ERROR("[AnymalLowLevelController] Unable to load ANYdrive Manager setup!");
    return false;
  }
  return true;
}

bool System::startup()
{
  MELO_INFO_STREAM("[AnymalLowLevelController]: Start starting up system.")

  std::lock_guard<std::mutex> lockGuard(anydriveMutex_);

  while (!isShutdownRequested_) {
    MELO_INFO("Initialize actuators.")
    if (!initActuators()) {
      MELO_ERROR("Cannot initialize actuators. --> Cannot startup actuators.")
      return false;
    }

    MELO_INFO("Try to start the ANYdrive Manager.")
    if (anydriveManager_->startup(1)) {
      MELO_INFO("[AnymalLowLevelController] ANYdrive Manager start successful.")
      break;
    } else {
      MELO_WARN("[AnymalLowLevelController] Unable to start the ANYdrive Manager.")
      anydriveManager_->getCommunicationManager()->shutdown();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  // Fast access anydrives vector.
  if (anydriveManager_) {
    for(auto actuatorKey : AD::getActuatorKeys()) {
      const auto actuatorEnum = actuatorKey.getEnum();
      anydrives_[actuatorEnum] = anydriveManager_->getAnydrive(actuatorKey.getName());
    }

    isActuatorSetup_ = true;
    isActuatorCommunicationEnabled_ = true;
  }

  MELO_INFO_STREAM("[AnymalLowLevelController]: Done starting up system.");
  return true;
}

void System::preCleanup()
{
  MELO_INFO_STREAM("[AnymalLowLevelController]: Start pre-cleaning up system.");

  isShutdownRequested_ = true;

  std::lock_guard<std::mutex> lockGuard(anydriveMutex_);
  if (anydriveManager_) {
    anydriveManager_->requestShutdown();
  }

  MELO_INFO_STREAM("[AnymalLowLevelController]: Done pre-cleaning up system.");
}

void System::cleanup()
{
  MELO_INFO_STREAM("[AnymalLowLevelController]: Start cleaning up system.");

  isActuatorSetup_ = false;
  isActuatorCommunicationEnabled_ = false;

  // Actuators.
  std::lock_guard<std::mutex> lockGuard(anydriveMutex_);
  if (anydriveManager_) {
    anydriveManager_->shutdown();
    MELO_INFO_STREAM("[AnymalLowLevelController]: Cleaned up ANYdrive Manager.");
  }

  MELO_INFO_STREAM("[AnymalLowLevelController]: Done cleaning up system.");
}

const System::AnydriveManagerPtr& System::getAnydriveManager()
{
  std::lock_guard<std::mutex> lockGuard(anydriveMutex_);
  return anydriveManager_;
}

const anydrive::AnydrivePtr& System::getAnydrive(AD::ActuatorEnum actuatorEnum) {
  std::lock_guard<std::mutex> lockGuard(anydriveMutex_);
  return anydrives_[actuatorEnum];
}

state_machine::Substates System::getActiveSubstates()
{
  return state_machine::Substates(getAnydrivesStateEnum());
}

state_machine::AnydrivesStateEnum System::getAnydrivesStateEnum()
{
  std::lock_guard<std::mutex> lockGuard(anydriveMutex_);
  if (!isActuatorSetup_ || !anydriveManager_->allDevicesAreConnected())
    return state_machine::AnydrivesStateEnum::Missing;
  else if (!anydriveManager_->noDeviceIsInFatalState())
    return state_machine::AnydrivesStateEnum::Fatal;
  else if (!anydriveManager_->noDeviceIsInErrorState())
    return state_machine::AnydrivesStateEnum::Error;
  else if (!anydriveManager_->allDevicesAreWithinJointPositionLimitsSdk())
    return state_machine::AnydrivesStateEnum::Error;
  else if (anydriveManager_->allDevicesAreInTheState(anydrive::fsm::StateEnum::ControlOp))
    return state_machine::AnydrivesStateEnum::Operational;
  else return state_machine::AnydrivesStateEnum::Other;
}

void System::enableActuatorCommunication(bool enable) {
  isActuatorCommunicationEnabled_ = enable;
}

bool System::isActutatorCommunicationEnabled() {
  return isActuatorCommunicationEnabled_;
}

bool System::isActuatorSetup() {
  return isActuatorSetup_;
}

void System::updateSensors(const any_worker::WorkerEvent& event) {
}

} // anymal_lowlevel_controller
