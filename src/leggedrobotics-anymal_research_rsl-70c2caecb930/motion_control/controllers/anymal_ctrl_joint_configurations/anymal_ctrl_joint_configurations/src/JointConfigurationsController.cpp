/*!
 * @file    JointConfigurationsController.cpp
 * @author  Alexander Reske
 * @brief   A controller that sends the robot into different joint configurations. E.g., the drives can be sent into default positions etc.
 */

// stl
#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

// boost
#include <boost/thread/lock_types.hpp>
#include <boost/thread/shared_mutex.hpp>

// anymal_motion_control
#include <anymal_motion_control/checks/JointPositionLimitsCheck.hpp>

// message_logger
#include <message_logger/message_logger.hpp>

// anymal_ctrl_joint_configurations
#include "anymal_ctrl_joint_configurations/JointConfigurationsController.hpp"

namespace anymal_ctrl_joint_configurations {

JointConfigurationsController::JointConfigurationsController()
    : Base(),
      state_(States::Standby),
      mode_("undefined"),
      badParameters_(false),
      maxJointVelocity_(0.0),
      maxAbsJointPositionError_(0.0),
      absJointPositionErrorTolerance_(0.0),
      timeout_(20.0),
      motionGeneration_(nullptr),
      currentJointPositions_(JointVector::Zero()),
      goalJointPositions_(JointVector::Zero()) {}

bool JointConfigurationsController::create() {
  motionGeneration_ = std::make_unique<MotionGeneration>();
  if (!motionGeneration_->create()) {
    MELO_ERROR("Could not create motion generation module.");
    return false;
  }

  // Add additional state checks
  anymal_motion_control::JointPositionLimitsCheck::JointPositionLimits jointPositionLimits;
  for (const auto& key : AD::getJointKeys()) {
    jointPositionLimits[key.getId()] = std::make_shared<const anymal_motion_control::NoLimitCheck>();
  }
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LF_KFE)] =
      std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-M_PI, 0.5 * M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RF_KFE)] =
      std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-M_PI, 0.5 * M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LH_KFE)] =
      std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-0.5 * M_PI, M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RH_KFE)] =
      std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-0.5 * M_PI, M_PI);
  getInitialStateChecker().addStateCheck("jointPositionLimits",
                                         std::make_shared<anymal_motion_control::JointPositionLimitsCheck>(jointPositionLimits));

  const std::string parameterFile = this->getParameterPath() + "/JointConfigurations" + (this->isRealRobot() ? "" : "Sim") + ".xml";
  if (!loadParams(parameterFile)) {
    return false;
  }

  setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::ACTION, getModes());

  motionGeneration_->setMaxJointVelocity(maxJointVelocity_);

  return true;
}

bool JointConfigurationsController::initialize() {
  if (!motionGeneration_->initialize()) {
    MELO_ERROR("Could not initialize motion generation module.");
    return false;
  }

  state_ = States::Standby;
  mode_ = "undefined";

  currentJointPositions_.setZero();
  goalJointPositions_.setZero();

  const std::string parameterFile = this->getParameterPath() + "/JointConfigurations" + (this->isRealRobot() ? "" : "Sim") + ".xml";
  return loadParams(parameterFile);
}

bool JointConfigurationsController::advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  std::lock_guard<std::mutex> controllerLock(controllerMutex_);
  {
    boost::unique_lock<boost::shared_mutex> lock(getStateMutex());
    currentJointPositions_ = getState().getAnymalModelPtr()->getState().getJointPositions().toImplementation();
  }
  switch (state_) {
    case States::Standby:
    case States::Success:
    case States::Preempted:
    case States::Error:
      if (!defaultAdvance(command, commandMutex)) {
        return false;
      }
      break;

    case States::Planning:
      if (!planningAdvance(command, commandMutex)) {
        return false;
      }
      state_ = States::Active;
      break;

    case States::Active:
      if (!motionGeneration_->advance(getTime().getTimeStep())) {
        MELO_ERROR("Could not advance motion generation module.");
        return false;
      }
      if (safetyCheckViolation()) {
        MELO_INFO_STREAM("Safety check violation: The actuators will freeze.");
        state_ = States::Error;
      }
      if (reachedGoal()) {
        MELO_INFO_STREAM("Reached goal joint position: The actuators will freeze.");
        state_ = States::Success;
      }
      if (state_ == States::Active) {
        if (!activeAdvance(command, commandMutex)) {
          return false;
        }
      }
      break;

    default:
      return false;
  }

  return true;
}

bool JointConfigurationsController::reset() {
  if (!motionGeneration_->reset()) {
    MELO_ERROR("Could not reset motion generation module.");
    return false;
  }
  return JointConfigurationsController::initialize();
}

bool JointConfigurationsController::preStop() {
  return true;
}

bool JointConfigurationsController::stop() {
  return true;
}

bool JointConfigurationsController::switchControllerMode(const std::string& mode) {
  std::lock_guard<std::mutex> controllerLock(controllerMutex_);
  state_ = (mode == "custom") ? States::Standby : States::Planning;
  mode_ = mode;
  return true;
}

const std::string JointConfigurationsController::getMode() const {
  return mode_;
}

const std::vector<std::string> JointConfigurationsController::getModes() const {
  std::vector<std::string> modes;
  modes.reserve(jointConfigurations_.size());
  for (const auto& jointConfiguration : jointConfigurations_) {
    modes.push_back(jointConfiguration.first);
  }
  return modes;
}

JointConfigurationsController::States& JointConfigurationsController::getControllerState() {
  return state_;
}

JointConfigurationsController::JointConfiguration& JointConfigurationsController::getCustomJointConfiguration() {
  return jointConfigurations_["custom"];
}

std::mutex& JointConfigurationsController::getControllerMutex() {
  return controllerMutex_;
}

anymal_motion_control::SwitchResult JointConfigurationsController::goToReferenceType(anymal_motion_control::ReferenceType referenceType) {
  switch (referenceType) {
    case (anymal_motion_control::ReferenceType::NA):
      return anymal_motion_control::SwitchResult::NA;
      break;
    case (anymal_motion_control::ReferenceType::POSE):
    case (anymal_motion_control::ReferenceType::TWIST):
      return anymal_motion_control::SwitchResult::NOT_FOUND;
      break;
    case (anymal_motion_control::ReferenceType::ACTION):
      return anymal_motion_control::SwitchResult::SWITCHED;
      break;
    default:
      return anymal_motion_control::SwitchResult::ERROR;
      break;
  }
}

void JointConfigurationsController::goToOperationMode(const std::string& operationMode,
                                                      anymal_motion_control::OperationModeAction* action) {
  std::vector<std::string> modes = getModes();
  if (std::any_of(modes.begin(), modes.end(), [&](std::string& mode) { return mode == operationMode; })) {
    MELO_INFO_STREAM("Received request to go to operation mode '" << operationMode << "'.");
    if (switchControllerMode(operationMode)) {
      auto start = getTime().now().toSeconds();
      while ((state_ == States::Standby) || (state_ == States::Planning) || (state_ == States::Active)) {
        if (getTime().now().toSeconds() - start >= timeout_) {
          MELO_WARN_STREAM("Reached timeout for operation mode '" << operationMode << "'.");
          std::lock_guard<std::mutex> controllerLock(controllerMutex_);
          state_ = States::Error;
          break;
        }
        if (action->isPreemptionRequested()) {
          MELO_WARN_STREAM("Received request to preempt operation mode '" << operationMode << "'.");
          std::lock_guard<std::mutex> controllerLock(controllerMutex_);
          state_ = States::Preempted;
          break;
        }
        getTime().sleep(0.01);
      }
      // There are four main reasons why the while loop terminates: States::Success, States::Preempted, States::Error or timeout
      // Anything else leads to a not assigned (NA) result
      if (state_ == States::Success) {
        auto result = anymal_motion_control::SwitchResult::SWITCHED;
        action->setSucceeded(result, "Successfully switched to and reached operation mode " + operationMode);
      } else if (state_ == States::Preempted) {
        auto result = anymal_motion_control::SwitchResult::ERROR;
        action->setPreempted(result, "Successfully switched to but received request to preempt operation mode " + operationMode);
      } else if (state_ == States::Error) {
        auto result = anymal_motion_control::SwitchResult::ERROR;
        action->setAborted(result, "Successfully switched to but encountered error in operation mode " + operationMode);
      } else {
        std::lock_guard<std::mutex> controllerLock(controllerMutex_);
        state_ = States::Error;
        auto result = anymal_motion_control::SwitchResult::NA;
        action->setAborted(result, "Successfully switched to but encountered unknown error in operation mode " + operationMode);
      }
    } else {
      auto result = anymal_motion_control::SwitchResult::ERROR;
      action->setAborted(result, "Could not switch to operation mode " + operationMode);
    }
  } else {
    auto result = anymal_motion_control::SwitchResult::NOT_FOUND;
    action->setAborted(result, "Could not find operation mode " + operationMode);
  }
}

bool JointConfigurationsController::defaultAdvance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  boost::unique_lock<boost::shared_mutex> lock(commandMutex);
  for (auto& actuatorCommand : command.getActuatorCommands()) {
    actuatorCommand.setMode(series_elastic_actuator::SeActuatorCommand::MODE_FREEZE);
  }
  return true;
}

bool JointConfigurationsController::planningAdvance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  {
    boost::unique_lock<boost::shared_mutex> lock(commandMutex);
    for (auto& actuatorCommand : command.getActuatorCommands()) {
      actuatorCommand.setMode(series_elastic_actuator::SeActuatorCommand::MODE_FREEZE);
    }
  }
  goalJointPositions_ = jointConfigurations_[mode_].jointPositions_;
  motionGeneration_->setStartJointPositions(currentJointPositions_);
  motionGeneration_->setGoalJointPositions(goalJointPositions_);
  motionGeneration_->planMotion();
  return true;
}

bool JointConfigurationsController::activeAdvance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  boost::unique_lock<boost::shared_mutex> lock(commandMutex);
  for (auto& actuatorCommand : command.getActuatorCommands()) {
    actuatorCommand.setMode(series_elastic_actuator::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS);
  }
  for (auto& actuatorCommand : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::HAA].values()) {
    actuatorCommand->setPidGains(gainsHaa_);
  }
  for (auto& actuatorCommand : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::HFE].values()) {
    actuatorCommand->setPidGains(gainsHfe_);
  }
  for (auto& actuatorCommand : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::KFE].values()) {
    actuatorCommand->setPidGains(gainsKfe_);
  }

  for (const auto key : AD::getActuatorKeys()) {
    const auto actuatorEnum = key.getEnum();
    const auto id = key.getId();
    const double desJointPosition = motionGeneration_->getDesJointPositions()[id];
    const double desJointVelocity = motionGeneration_->getDesJointVelocities()[id];
    command.getActuatorCommands()[actuatorEnum].setJointPosition(desJointPosition);
    command.getActuatorCommands()[actuatorEnum].setJointVelocity(desJointVelocity);
    command.getActuatorCommands()[actuatorEnum].setJointTorque(0.0);
  }
  return true;
}

bool JointConfigurationsController::safetyCheckViolation() {
  if (badParameters_) {
    MELO_WARN_STREAM("Safety check violation: A bad parameter has been chosen (see warnings above).");
    return true;
  }
  if (detectedContact()) {
    MELO_WARN_STREAM("Safety check violation: Detected contacts! Only use this controller if the legs are not in contact with the ground.");
    return true;
  }
  if (badTracking()) {
    MELO_WARN_STREAM("Safety check violation: The tracking error is too large! Check for potential (self-)collision or blocking joints.");
    return true;
  }
  return false;
}

bool JointConfigurationsController::detectedContact() {
  if (!jointConfigurations_[mode_].contact_) {
    return false;
  }
  // check contact
  bool contact;
  {
    boost::unique_lock<boost::shared_mutex> lock(getStateMutex());
    auto& contactContainer = getState().getAnymalModelPtr()->getContactContainer();
    contact = std::any_of(contactContainer.begin(), contactContainer.end(),
                          [](auto contact) { return contact->getState() != AD::ContactStateEnum::OPEN; });
  }
  // special case: detected contact in mode rest
  if (contact && (mode_ == "rest")) {
    bool closeToRest = true;
    const JointVector absJointPositionErrors = (currentJointPositions_ - goalJointPositions_).cwiseAbs();
    for (const auto& key : AD::getJointKeys()) {
      const std::string jointNode = AD::mapKeyEnumToKeyName(AD::mapJointEnumToJointNodeEnum::at(key.getEnum()));
      if (absJointPositionErrors[key.getId()] > maxAbsJointPositionDeviationFromRest_[jointNode]) {
        MELO_WARN_STREAM("Contact detection: note that " << jointNode << " is " << absJointPositionErrors[key.getId()] << " > "
                                                         << maxAbsJointPositionDeviationFromRest_[jointNode] << " [rad] away from rest");
        closeToRest = false;
        break;
      }
    }
    // only report contact if detected contact and at least one joint is far from rest configuration
    contact = contact && !closeToRest;
  }
  return contact;
}

bool JointConfigurationsController::badTracking() {
  // check tracking
  JointVector desJointPositions = motionGeneration_->getDesJointPositions();
  double maxAbsJointPositionError = (currentJointPositions_ - desJointPositions).cwiseAbs().maxCoeff();
  return maxAbsJointPositionError > maxAbsJointPositionError_;
}

bool JointConfigurationsController::reachedGoal() {
  double maxAbsJointPositionError = (currentJointPositions_ - goalJointPositions_).cwiseAbs().maxCoeff();
  return (motionGeneration_->getTime() > motionGeneration_->getDuration()) && (maxAbsJointPositionError <= absJointPositionErrorTolerance_);
}

bool JointConfigurationsController::loadParams(const std::string& filename) {
  badParameters_ = false;
  jointConfigurations_.clear();
  maxAbsJointPositionDeviationFromRest_.clear();

  bool verbose = true;

  TiXmlDocument document(filename);
  if (!document.LoadFile()) {
    MELO_ERROR_STREAM("Could not load parameter file: " << filename);
    return false;
  }

  TiXmlHandle documentHandle(&document);

  TiXmlHandle rootHandle(documentHandle);
  if (!tinyxml_tools::getChildHandle(rootHandle, documentHandle, "JointConfigurationsController", verbose)) {
    return false;
  }

  TiXmlHandle gainsHandle(rootHandle);
  if (!tinyxml_tools::getChildHandle(gainsHandle, rootHandle, "Gains", verbose)) {
    return false;
  }
  TiXmlHandle configsHandle(rootHandle);
  if (!tinyxml_tools::getChildHandle(configsHandle, rootHandle, "JointConfigurations", verbose)) {
    return false;
  }
  TiXmlHandle limitsHandle(rootHandle);
  if (!tinyxml_tools::getChildHandle(limitsHandle, rootHandle, "Limits", verbose)) {
    return false;
  }

  // gains
  TiXmlHandle driveHandle(gainsHandle);
  if (!tinyxml_tools::getChildHandle(driveHandle, gainsHandle, "HAA", verbose)) {
    return false;
  }
  if (!loadGainParam(driveHandle, gainsHaa_)) {
    return false;
  }
  if (!tinyxml_tools::getChildHandle(driveHandle, gainsHandle, "HFE", verbose)) {
    return false;
  }
  if (!loadGainParam(driveHandle, gainsHfe_)) {
    return false;
  }
  if (!tinyxml_tools::getChildHandle(driveHandle, gainsHandle, "KFE", verbose)) {
    return false;
  }
  if (!loadGainParam(driveHandle, gainsKfe_)) {
    return false;
  }

  // joint configurations
  std::vector<TiXmlElement*> configElements;
  if (!tinyxml_tools::getChildElements(configElements, configsHandle, "JointConfiguration")) {
    return false;
  }
  for (const auto configElement : configElements) {
    if (configElement == nullptr) {
      return false;
    }

    std::string name;
    JointConfiguration jointConfiguration;
    jointConfigurations_.insert(std::pair<std::string, JointConfiguration>("custom", jointConfiguration));
    if (!tinyxml_tools::loadParameter(name, configElement, "name", "undefined")) {
      return false;
    }
    if (!loadConfigurationParam(configElement, jointConfiguration)) {
      return false;
    }
    jointConfigurations_.insert(std::pair<std::string, JointConfiguration>(name, jointConfiguration));
  }

  // limits
  TiXmlHandle limitHandle(limitsHandle);
  if (!tinyxml_tools::getChildHandle(limitHandle, limitsHandle, "MaxJointVelocity", verbose)) {
    return false;
  }
  if (!loadLimitParam(limitHandle, maxJointVelocity_)) {
    return false;
  }
  if (maxJointVelocity_ <= 0.0) {
    badParameters_ = true;
    MELO_WARN_STREAM("Bad parameter: Maximum joint velocity is smaller or equal to zero. Actuators will freeze.");
    maxJointVelocity_ = 0.0;
  }
  if (!tinyxml_tools::getChildHandle(limitHandle, limitsHandle, "MaxAbsJointPositionError", verbose)) {
    return false;
  }
  if (!loadLimitParam(limitHandle, maxAbsJointPositionError_)) {
    return false;
  }
  if (maxAbsJointPositionError_ <= 0.0) {
    badParameters_ = true;
    MELO_WARN_STREAM("Bad parameter: Maximum absolute joint position error is smaller or equal to zero. Actuators will freeze.");
    maxAbsJointPositionError_ = 0.0;
  }
  if (!tinyxml_tools::getChildHandle(limitHandle, limitsHandle, "AbsJointPositionErrorTolerance", verbose)) {
    return false;
  }
  if (!loadLimitParam(limitHandle, absJointPositionErrorTolerance_)) {
    return false;
  }
  if (absJointPositionErrorTolerance_ <= 0.0) {
    badParameters_ = true;
    MELO_WARN_STREAM("Bad parameter: Absolute joint position error tolerance is smaller or equal to zero. Actuators will freeze.");
    absJointPositionErrorTolerance_ = 0.0;
  }
  if (!tinyxml_tools::getChildHandle(limitHandle, limitsHandle, "MaxAbsJointPositionDeviationFromRest", verbose)) {
    return false;
  }
  for (const auto& key : AD::getJointNodeKeys()) {
    double limit;
    const auto name = std::string(key.getName());
    if (!tinyxml_tools::loadParameter(limit, limitHandle, name)) {
      MELO_ERROR_STREAM("Parameter not found: Maximum absolute joint position deviation from rest for " << name.c_str()
                                                                                                        << ". Controller will not load.");
      return false;
    }
    if (limit <= 0.0) {
      badParameters_ = true;
      MELO_WARN_STREAM("Bad parameter: Maximum absolute joint position deviation from rest for "
                       << name.c_str() << " is smaller or equal to zero. Actuators will freeze.");
      limit = 0.0;
    }
    maxAbsJointPositionDeviationFromRest_.insert(std::pair<std::string, double>(name, limit));
  }

  return true;
}

bool JointConfigurationsController::loadGainParam(const TiXmlHandle& driveHandle,
                                                  series_elastic_actuator::SeActuatorCommand::PidGains& pidGains) {
  if (!tinyxml_tools::loadParameter(pidGains.pGain_, driveHandle, "pGain", 0.0)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(pidGains.iGain_, driveHandle, "iGain", 0.0)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(pidGains.dGain_, driveHandle, "dGain", 0.0)) {
    return false;
  }
  return true;
}

bool JointConfigurationsController::loadConfigurationParam(const TiXmlHandle& configurationHandle, JointConfiguration& jointConfiguration) {
  bool verbose = true;

  // checks
  TiXmlHandle checksHandle(configurationHandle);
  if (!tinyxml_tools::getChildHandle(checksHandle, configurationHandle, "Checks", verbose)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(jointConfiguration.contact_, checksHandle, "contact", true)) {
    return false;
  }

  // joint positions
  TiXmlHandle jointPositionsHandle(configurationHandle);
  JointVector jointPositions = JointVector::Zero();
  if (!tinyxml_tools::getChildHandle(jointPositionsHandle, configurationHandle, "JointPositions", verbose)) {
    return false;
  }
  for (const auto key : AD::getJointKeys()) {
    const auto id = key.getId();
    const auto name = std::string(key.getName());
    if (!tinyxml_tools::loadParameter(jointPositions[id], jointPositionsHandle, name, 0.0)) {
      return false;
    }
  }
  jointConfiguration.jointPositions_ = jointPositions;

  return true;
}

bool JointConfigurationsController::loadLimitParam(const TiXmlHandle& limitHandle, double& limit) {
  return tinyxml_tools::loadParameter(limit, limitHandle, "value", 0.0);
}

} /* namespace anymal_ctrl_joint_configurations */
