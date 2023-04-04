#include <chrono>

#include "anydrive/Anydrive.hpp"

namespace anydrive {

Anydrive::Anydrive() : stateMachine_(*this), statuswordRequested_(false), commandIsStaged_(false), wasWithinJointPositionLimitsSdk_(true) {}

bool Anydrive::loadSetup(const setup::AnydrivePtr& setup) {
  name_ = setup->name_;
  configuration_ = setup->configuration_;
  return true;
}

std::string Anydrive::getName() const {
  return name_;
}

configuration::Configuration& Anydrive::getConfiguration() {
  return configuration_;
}

const configuration::Configuration& Anydrive::getConfiguration() const {
  return configuration_;
}

void Anydrive::setCommunicationInterface(const communication::CommunicationInterfaceBasePtr& communicationInterface) {
  communicationInterface_ = communicationInterface;
}

const communication::CommunicationInterfaceBasePtr& Anydrive::getCommunicationInterface() const {
  return communicationInterface_;
}

void Anydrive::addReadingCb(const ReadingCb& readingCb, const int priority) {
  readingCbs_.insert({priority, readingCb});
}

void Anydrive::addErrorCb(const ErrorCb& errorCb, const int priority) {
  errorCbs_.insert({priority, errorCb});
}

void Anydrive::addErrorRecoveredCb(const ErrorRecoveredCb& errorRecoveredCb, const int priority) {
  errorRecoveredCbs_.insert({priority, errorRecoveredCb});
}

void Anydrive::addFatalCb(const FatalCb& fatalCb, const int priority) {
  fatalCbs_.insert({priority, fatalCb});
}

void Anydrive::addFatalRecoveredCb(const FatalRecoveredCb& fatalRecoveredCb, const int priority) {
  fatalRecoveredCbs_.insert({priority, fatalRecoveredCb});
}

void Anydrive::addDeviceDisconnectedCb(const DeviceDisconnectedCb& deviceDisconnectedCb, const int priority) {
  deviceDisconnectedCbs_.insert({priority, deviceDisconnectedCb});
}

void Anydrive::addDeviceReconnectedCb(const DeviceReconnectedCb& deviceReconnectedCb, const int priority) {
  deviceReconnectedCbs_.insert({priority, deviceReconnectedCb});
}

void Anydrive::startupWithoutCommunication() {
  ANYDRIVE_NAMED_DEBUG("Starting up ANYdrive without communication ...");

  const fsm::StateEnum goalStateEnumStartup = getConfiguration().getGoalStateEnumStartup();
  if (goalStateEnumStartup == fsm::StateEnum::NA) {
    ANYDRIVE_NAMED_DEBUG("No startup goal state has been set.")
  } else {
    stateMachine_.setGoalStateEnum(goalStateEnumStartup);
  }
  if (getConfiguration().getAutoStageLastCommand()) {
    stageFreeze();
  }
  //  if (!communicationInterface_->startup())
  //    shutdownWithoutCommunication();
}

void Anydrive::startupWithCommunication() {
  ANYDRIVE_NAMED_DEBUG("Starting up ANYdrive with communication ...");

  /*
  // Get the minimal and the newest firmware version.
  Version minimal(MINIMAL_ANYDRIVE_FIRMWARE_VERSION);
  Version newest(NEWEST_ANYDRIVE_FIRMWARE_VERSION);
  if (newest < minimal)
  {
    ANYDRIVE_NAMED_ERROR("The newest firmware version is lower than the minimal firmware version.");
    return;
  }

  // Check the firmware version.
  Version current;
  if (!getDriveInfoFirmwareVersion(current))
  {
    ANYDRIVE_NAMED_ERROR("Could not get firmware version.");
  }
  else if (current < minimal)
  {
    ANYDRIVE_NAMED_ERROR("The firmware on your ANYdrive is not supported anymore. Please update it.");
    ANYDRIVE_NAMED_ERROR("Current version: " << current);
    ANYDRIVE_NAMED_ERROR("Minimal version: " << minimal);
    ANYDRIVE_NAMED_ERROR("Newest version:  " << newest);
  }
  else if (current < newest)
  {
    ANYDRIVE_NAMED_WARN("The firmware of your ANYdrive is outdated. Please consider updating it.");
    ANYDRIVE_NAMED_WARN("Current version: " << current);
    ANYDRIVE_NAMED_WARN("Newest version:  " << newest);
  }
  else if (current == newest)
  {
    ANYDRIVE_NAMED_DEBUG("The firmware of your ANYdrive is up to date.");
    ANYDRIVE_NAMED_DEBUG("Current version: " << current);
  }
  else // (current > newest)
  {
    ANYDRIVE_NAMED_WARN("The firmware of your ANYdrive is unknown.");
    ANYDRIVE_NAMED_WARN("Current version: " << current);
    ANYDRIVE_NAMED_WARN("Newest version:  " << newest);
  }
  */
}

void Anydrive::updateProcessReading() {
  // Get reading.
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    communicationInterface_->getReading(reading_);

    // Update statusword.
    Statusword statusword(reading_.getState().getStatusword());
    setStatusword(statusword);
    statuswordRequested_ = false;

    // Check joint position SDK limits.
    if (getActiveStateEnum() == fsm::StateEnum::ControlOp) {
      if (wasWithinJointPositionLimitsSdk_) {
        if (!isWithinJointPositionLimitsSdk()) {
          wasWithinJointPositionLimitsSdk_ = false;
          ANYDRIVE_LOGGED_NAMED_ERROR("Joint position exceeded SDK limits.");
          errorCb();
        }
      } else {
        if (isWithinJointPositionLimitsSdk()) {
          wasWithinJointPositionLimitsSdk_ = true;
          ANYDRIVE_NAMED_INFO("Joint position is again within SDK limits.");
          //        errorRecoveredCb(); // TODO(remo) enable?
        }
      }
    }

    // Log if activated.
    if (logger_.logIsActive()) {
      logger_.addDataToLog(reading_);
    }

    // External reading callbacks.
    for (const auto& readingCb : readingCbs_) {
      readingCb.second(getName(), reading_);
    }
  }

  if (deviceIsMissing()) {
    Statusword statusword;
    statusword.setStateEnum(fsm::StateEnum::DeviceMissing);
    setStatusword(statusword);
  } else {
    if (statusword_.isEmpty() /* || statusword_.getAge() > getConfiguration().getMaxStatuswordAge()*/)  // TODO(remo) Delete statusword
                                                                                                        // request over SDO?
    {
      // Device is not missing but statusword is empty or outdated.
      requestAndSetStatusword();
    }
    if (statusword_.isEmpty()) {
      // Device is not missing but statusword is still empty.
      return;
    }
  }

  const fsm::StateEnum activeState = statusword_.getStateEnum();
  if (activeState == fsm::StateEnum::NA) {
    ANYDRIVE_NAMED_DEBUG("The FSM state is not available.");
    return;
  }
  stateMachine_.updateActiveState(activeState);
}

void Anydrive::updateSendStagedCommand() {
  if (!commandIsStaged_) {
    return;
  }

  // Lock the staged command for the entire duration of this function, so that a potential
  // external stage command always overwrites the auto-staged command.
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);

  // Check the age of the command.
  const double commandAge = stagedCommand_.getStamp().getElapsedTime().toSeconds();
  const double maxCommandAge = getConfiguration().getMaxCommandAge();
  if (commandAge > maxCommandAge) {
    ANYDRIVE_ERROR("Command is older than max age (" << commandAge << " s > " << maxCommandAge << " s).");
    return;
  }

  // Send the command.
  communicationInterface_->setCommand(stagedCommand_);
  commandIsStaged_ = false;

  // Auto-stage the last command if configured to do so.
  if (getConfiguration().getAutoStageLastCommand()) {
    Command nextCommand = stagedCommand_;
    nextCommand.setStamp(any_measurements::Time::NowWallClock());
    stageCommand(nextCommand);
  }
}

void Anydrive::shutdownWithCommunication() {
  shutdownWithCommunication(true, 5.0, 100.0);
}

void Anydrive::shutdownWithCommunication(const bool reachState, const double timeout, const double checkingFrequency) {
  ANYDRIVE_NAMED_DEBUG("Preparing to shut down ANYdrive ...");
  fsm::StateEnum goalStateEnumShutdown = getConfiguration().getGoalStateEnumShutdown();
  if (deviceIsMissing()) {
    ANYDRIVE_NAMED_WARN("Device is missing, cannot transition to shutdown state '" << fsm::stateEnumToName(goalStateEnumShutdown) << "'.");
  } else if (getActiveStateEnum() == fsm::StateEnum::Error || getActiveStateEnum() == fsm::StateEnum::Fatal) {
    // Nothing should be done.
    return;
  } else {
    // Special case if the drive is in ControlOp.
    if (getActiveStateEnum() == fsm::StateEnum::ControlOp) {
      stageFreeze();

      // The drive should not remain in ControlOp.
      if (goalStateEnumShutdown == fsm::StateEnum::NA) {
        goalStateEnumShutdown = fsm::StateEnum::MotorOp;
      }
    }

    // Reach the shutdown goal state if it is set.
    if (goalStateEnumShutdown == fsm::StateEnum::NA) {
      ANYDRIVE_NAMED_DEBUG("No shutdown goal state has been set.");
    } else {
      if (setGoalStateEnum(goalStateEnumShutdown, reachState, timeout, checkingFrequency))
        ANYDRIVE_NAMED_DEBUG("Shutdown goal state has been reached.")
      else
        ANYDRIVE_LOGGED_NAMED_ERROR("Shutdown goal state has not been reached within timeout.")
    }
  }
}

void Anydrive::shutdownWithoutCommunication() {
  ANYDRIVE_NAMED_DEBUG("Shutting down ANYdrive ...");
  //  communicationInterface_->shutdown();
}

fsm::StateEnum Anydrive::getActiveStateEnum() const {
  return stateMachine_.getActiveStateEnum();
}

fsm::StateEnum Anydrive::getGoalStateEnum() const {
  return stateMachine_.getGoalStateEnum();
}

bool Anydrive::goalStateHasBeenReached() const {
  return stateMachine_.goalStateHasBeenReached();
}

bool Anydrive::setGoalStateEnum(const fsm::StateEnum goalStateEnum, const bool reachState, const double timeout,
                                const double checkingFrequency) {
  stateMachine_.setGoalStateEnum(goalStateEnum);

  if (!reachState) {
    return true;
  }

  const double timeStep = 1.0 / checkingFrequency;
  double timeSlept = 0.0;
  while (true) {
    if (timeSlept > timeout) {
      return false;
    }
    if (goalStateHasBeenReached()) {
      return true;
    }
    threadSleep(timeStep);
    timeSlept += timeStep;
  }
}

void Anydrive::clearGoalStateEnum() {
  stateMachine_.clearGoalStateEnum();
}

void Anydrive::errorCb() {
  for (const auto& errorCb : errorCbs_) {
    errorCb.second(getName());
  }
}

void Anydrive::errorRecoveredCb() {
  clearGoalStateEnum();
  for (const auto& errorRecoveredCb : errorRecoveredCbs_) {
    errorRecoveredCb.second(getName());
  }
}

bool Anydrive::deviceIsInErrorState() {
  return getStatusword().getStateEnum() == fsm::StateEnum::Error;
}

void Anydrive::fatalCb() {
  for (const auto& fatalCb : fatalCbs_) {
    fatalCb.second(getName());
  }
}

void Anydrive::fatalRecoveredCb() {
  clearGoalStateEnum();
  for (const auto& fatalRecoveredCb : fatalRecoveredCbs_) {
    fatalRecoveredCb.second(getName());
  }
}

bool Anydrive::deviceIsInFatalState() {
  return getStatusword().getStateEnum() == fsm::StateEnum::Fatal;
}

void Anydrive::deviceDisconnectedCb() {
  statuswordRequested_ = false;
  clearGoalStateEnum();
  for (const auto& deviceDisconnectedCb : deviceDisconnectedCbs_) {
    deviceDisconnectedCb.second(getName());
  }

  // Set statusword and reading accordingly.
  //  statusword_.resetData();
  ReadingExtended reading;
  const any_measurements::Time stamp = any_measurements::Time::NowWallClock();

  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    StateExtended& state = reading_.getState();
    state.setStamp(stamp);
    state.setStatusword(statusword_);
    if (getConfiguration().getSetReadingToNanOnDisconnect()) {
      // State.
      state.setCurrent(nan_);
      state.setGearPosition(nan_);
      state.setGearVelocity(nan_);
      state.setJointPosition(nan_);
      state.setJointVelocity(nan_);
      state.setJointAcceleration(nan_);
      state.setJointTorque(nan_);

      // Extended state.
      state.setMotorPosition(nan_);
      state.setMotorVelocity(nan_);
      state.setGearPositionTicks(0);
      state.setJointPositionTicks(0);
      state.setTemperature(nan_);
      state.setVoltage(nan_);
      state.setTimestamp(0);
      state.setDesiredCurrentD(nan_);
      state.setMeasuredCurrentD(nan_);
      state.setDesiredCurrentQ(nan_);
      state.setMeasuredCurrentQ(nan_);
      state.setMeasuredCurrentPhaseU(nan_);
      state.setMeasuredVoltagePhaseU(nan_);
      state.setMeasuredCurrentPhaseV(nan_);
      state.setMeasuredVoltagePhaseV(nan_);
      state.setMeasuredCurrentPhaseW(nan_);
      state.setMeasuredVoltagePhaseW(nan_);
    }
    reading = reading_;
  }

  // External reading callbacks.
  for (const auto& readingCb : readingCbs_) {
    readingCb.second(getName(), reading);
  }
}

void Anydrive::deviceReconnectedCb() {
  setGoalStateEnum(getConfiguration().getGoalStateEnumStartup());
  for (const auto& deviceReconnectedCb : deviceReconnectedCbs_) {
    deviceReconnectedCb.second(getName());
  }
}

bool Anydrive::deviceIsMissing() const {
  return communicationInterface_->deviceIsMissing();
}

bool Anydrive::isWithinJointPositionLimitsSdk() const {
  common::Limits limits;
  getJointPositionLimitsSdk(limits);
  return limits.liesWithin(getReading().getState().getJointPosition());
}

bool Anydrive::isWithinJointPositionLimitsSoft() const {
  return getStatusword().hasErrorJointPositionLimitsSoft();
}

bool Anydrive::isWithinJointPositionLimitsHard() const {
  return getStatusword().hasFatalJointPositionLimitsHard();
}

bool Anydrive::getDriveInfo(DriveInfo& driveInfo) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the drive info.");
    return false;
  }

  bool success = true;
  success &= getDriveInfoSerialNumber(driveInfo.getSerialNumber());
  success &= getDriveModel(driveInfo.getModel());
  success &= getDriveInfoName(driveInfo.getName());
  success &= getDriveInfoId(driveInfo.getId());
  success &= getDriveInfoBootloaderVersion(driveInfo.getBootloaderVersion());
  success &= getDriveInfoFirmwareVersion(driveInfo.getFirmwareVersion());
  return success;
}

bool Anydrive::getDriveModel(std::string& model) const {
  return communicationInterface_->getDriveModel(model);
}

bool Anydrive::getDriveInfoSerialNumber(std::string& serialNumber) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the drive info serial number.");
    return false;
  }

  return communicationInterface_->getDriveInfoSerialNumber(serialNumber);
}

bool Anydrive::setDriveInfoSerialNumber(const std::string& serialNumber) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the drive info serial number.");
    return false;
  }

  return communicationInterface_->setDriveInfoSerialNumber(serialNumber);
}

bool Anydrive::getDriveInfoName(std::string& name) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the drive info name.");
    return false;
  }

  return communicationInterface_->getDriveInfoName(name);
}

bool Anydrive::setDriveInfoName(const std::string& name) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the drive info name.");
    return false;
  }

  return communicationInterface_->setDriveInfoName(name);
}

bool Anydrive::getDriveInfoId(uint16_t& id) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the drive info ID.");
    return false;
  }

  return communicationInterface_->getDriveInfoId(id);
}

bool Anydrive::setDriveInfoId(const uint16_t id) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the drive info ID.");
    return false;
  }

  return communicationInterface_->setDriveInfoId(id);
}

bool Anydrive::getDriveInfoBootloaderVersion(common::Version& bootloaderVersion) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the drive info bootloader version.");
    return false;
  }

  return communicationInterface_->getDriveInfoBootloaderVersion(bootloaderVersion);
}

bool Anydrive::setDriveInfoBootloaderVersion(const common::Version& bootloaderVersion) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the drive info bootloader version.");
    return false;
  }

  return communicationInterface_->setDriveInfoBootloaderVersion(bootloaderVersion);
}

bool Anydrive::getDriveInfoFirmwareVersion(common::Version& firmwareVersion) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the drive info firmware version.");
    return false;
  }

  return communicationInterface_->getDriveInfoFirmwareVersion(firmwareVersion);
}

bool Anydrive::getDriveFirmwareInfo(common::FirmwareInfo& firmwareInfo) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the firmware info.");
    return false;
  }

  return communicationInterface_->getDriveFirmwareInfo(firmwareInfo);
}

bool Anydrive::getGearboxRatio(uint32_t& ratio) {
  return communicationInterface_->getGearboxRatio(ratio);
}

bool Anydrive::eraseFlashStorage() {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to erase flash storage.");
    return false;
  }

  return communicationInterface_->eraseFlashStorage();
}

bool Anydrive::resetFlashStorageSections(const uint8_t flashStorageSections) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to reset flash storage sections.");
    return false;
  }

  return communicationInterface_->resetFlashStorageSections(flashStorageSections);
}

bool Anydrive::getCalibrationState(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                   calibration::CalibrationState& calibrationState) const {
  return communicationInterface_->getCalibrationState(calibrationTypeEnum, calibrationState);
}

bool Anydrive::getCalibration(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                              calibration::parameter::Calibration& calibration) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get calibration values.");
    return false;
  }

  bool success = true;
  success &= getCalibrationMotorEncoderOffset(calibrationTypeEnum, calibration.motorEncoderOffset_);
  success &= getCalibrationMotorEncoderParameters(calibrationTypeEnum, calibration.motorEncoderParameters_);
  success &= getCalibrationGearJointEncoderOffset(calibrationTypeEnum, calibration.gearJointEncoderOffset_);
  success &= getCalibrationGearAndJointEncoderHoming(calibrationTypeEnum, calibration.gearAndJointEncoderHoming_);
  success &= getCalibrationImuGyroscopeDcBias(calibrationTypeEnum, calibration.imuGyroscopeDcBias_);
  success &= getCalibrationSpringStiffness(calibrationTypeEnum, calibration.springStiffness_);
  success &= getCalibrationFrictionEstimation(calibrationTypeEnum, calibration.frictionEstimation_);
  return success;
}

bool Anydrive::getCalibrationMotorEncoderOffset(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                calibration::parameter::MotorEncoderOffset& motorEncoderOffset) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get motor encoder offset calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationMotorEncoderOffset(calibrationTypeEnum, motorEncoderOffset);
}

bool Anydrive::getCalibrationMotorEncoderParameters(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                    calibration::parameter::MotorEncoderParameters& motorEncoderParameters) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get motor encoder parameters calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationMotorEncoderParameters(calibrationTypeEnum, motorEncoderParameters);
}

bool Anydrive::getCalibrationGearJointEncoderOffset(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                    calibration::parameter::GearJointEncoderOffset& gearJointEncoderOffset) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get gear/joint encoder offset calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationGearJointEncoderOffset(calibrationTypeEnum, gearJointEncoderOffset);
}

bool Anydrive::getCalibrationGearJointEncoderOffsetConstant(int32_t& constant) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get gear/joint encoder offset calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationGearJointEncoderOffsetConstant(constant);
}

bool Anydrive::setCalibrationGearJointEncoderOffsetConstant(const int32_t constant) {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to set gear/joint encoder offset calibration values.");
    return false;
  }

  return communicationInterface_->setCalibrationGearJointEncoderOffsetConstant(constant);
}

bool Anydrive::getCalibrationGearAndJointEncoderHoming(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                       calibration::parameter::GearAndJointEncoderHoming& gearAndJointEncoderHoming) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get gear and joint encoder homing calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationGearAndJointEncoderHoming(calibrationTypeEnum, gearAndJointEncoderHoming);
}

bool Anydrive::getCalibrationImuGyroscopeDcBias(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                calibration::parameter::ImuGyroscopeDcBias& imuGyroscopeDcBias) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get IMU gyroscope dc bias calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationImuGyroscopeDcBias(calibrationTypeEnum, imuGyroscopeDcBias);
}

bool Anydrive::setCalibrationSpringStiffness(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                             const calibration::parameter::SpringStiffness& springStiffness) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to set spring stiffness calibration values.");
    return false;
  }

  return communicationInterface_->setCalibrationSpringStiffness(calibrationTypeEnum, springStiffness);
}

bool Anydrive::getCalibrationSpringStiffness(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                             calibration::parameter::SpringStiffness& springStiffness) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get spring stiffness calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationSpringStiffness(calibrationTypeEnum, springStiffness);
}

bool Anydrive::setCalibrationFrictionEstimation(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                const calibration::parameter::FrictionEstimation& frictionEstimation) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to set friction estimation calibration values.");
    return false;
  }

  return communicationInterface_->setCalibrationFrictionEstimation(calibrationTypeEnum, frictionEstimation);
}

bool Anydrive::getCalibrationFrictionEstimation(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                calibration::parameter::FrictionEstimation& frictionEstimation) const {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to get friction estimation calibration values.");
    return false;
  }

  return communicationInterface_->getCalibrationFrictionEstimation(calibrationTypeEnum, frictionEstimation);
}

bool Anydrive::resetCustomCalibrationsToFactory(const calibration::CalibrationState calibrationState) {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to reset custom calibrations to factory.");
    return false;
  }

  return communicationInterface_->resetCustomCalibrationsToFactory(calibrationState);
}

bool Anydrive::writeFactoryCalibration() {
  if (getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in calibrate state to write factory calibration.");
    return false;
  }

  return communicationInterface_->writeFactoryCalibration();
}

bool Anydrive::waitForCalibrationDone(double timeout, double checkingFrequency) const {
  const double timeStep = 1.0 / checkingFrequency;
  double timeSlept = 0.0;
  bool calibrationIsRunning = true;
  while (true) {
    if (timeSlept > timeout) {
      return false;
    }
    communicationInterface_->calibrationIsRunning(calibrationIsRunning);
    if (!calibrationIsRunning) {
      return true;
    }
    threadSleep(timeStep);
    timeSlept += timeStep;
  }
}

bool Anydrive::getMaxCurrent(double& maxCurrent) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get max current.");
    return false;
  }

  return communicationInterface_->getMaxCurrent(maxCurrent);
}

bool Anydrive::setMaxCurrent(const double maxCurrent) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set max current.");
    return false;
  }

  if (!communicationInterface_->setMaxCurrent(maxCurrent)) {
    return false;
  }
  getConfiguration().setMaxCurrent(maxCurrent);
  return true;
}

bool Anydrive::getMaxFreezeCurrent(double& current) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the freeze current limit.");
    return false;
  }

  return communicationInterface_->getMaxFreezeCurrent(current);
}

bool Anydrive::setMaxFreezeCurrent(const double current) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the freeze current limit.");
    return false;
  }

  if (!communicationInterface_->setMaxFreezeCurrent(current)) {
    return false;
  }
  getConfiguration().setMaxFreezeCurrent(current);
  return true;
}

bool Anydrive::clearLoggedData() {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to clear logged data.");
    return false;
  }
  return communicationInterface_->clearLoggedData();
}

bool Anydrive::refreshLoggedData() {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to refresh logged data.");
    return false;
  }
  return communicationInterface_->refreshLoggedData();
}

bool Anydrive::getNumHistogramBins(int& numHistogramBins) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to read histogram bin number.");
    return false;
  }
  return communicationInterface_->getNumHistogramBins(numHistogramBins);
}

bool Anydrive::readHistograms(std::vector<uint64_t>& histTorque, std::vector<uint64_t>& histVelocity, std::vector<uint64_t>& histCurrent,
                              std::vector<uint64_t>& histTemperature) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to read histograms.");
    return false;
  }
  return communicationInterface_->readHistograms(histTorque, histVelocity, histCurrent, histTemperature);
}

bool Anydrive::readOperatingTimes(uint64_t& timeTotal, uint64_t& timeActive) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to read histograms.");
    return false;
  }
  return communicationInterface_->readOperatingTimes(timeTotal, timeActive);
}

bool Anydrive::readJointTravel(double& jointTravel) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to read histograms.");
    return false;
  }
  return communicationInterface_->readJointTravel(jointTravel);
}

bool Anydrive::readHistogramEdges(float& torque_upper, float& torque_lower, float& velocity_upper, float& velocity_lower,
                                  float& current_upper, float& current_lower, float& temperature_upper, float& temperature_lower) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to read histograms.");
    return false;
  }
  return communicationInterface_->readHistogramEdges(torque_upper, torque_lower, velocity_upper, velocity_lower, current_upper,
                                                     current_lower, temperature_upper, temperature_lower);
}

bool Anydrive::getMaxMotorVelocity(double& maxMotorVelocity) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get max motor velocity.");
    return false;
  }

  return communicationInterface_->getMaxMotorVelocity(maxMotorVelocity);
}

bool Anydrive::setMaxMotorVelocity(const double maxMotorVelocity) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set max motor velocity.");
    return false;
  }

  if (!communicationInterface_->setMaxMotorVelocity(maxMotorVelocity)) {
    return false;
  }
  getConfiguration().setMaxMotorVelocity(maxMotorVelocity);
  return true;
}

bool Anydrive::getMaxJointTorque(double& maxJointTorque) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get max joint torque.");
    return false;
  }

  return communicationInterface_->getMaxJointTorque(maxJointTorque);
}

bool Anydrive::setMaxJointTorque(const double maxJointTorque) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set max joint torque.");
    return false;
  }

  if (!communicationInterface_->setMaxJointTorque(maxJointTorque)) {
    return false;
  }
  getConfiguration().setMaxJointTorque(maxJointTorque);
  return true;
}

bool Anydrive::getCurrentIntegratorSaturation(double& saturation) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the integrator saturation of the current controller.");
    return false;
  }

  return communicationInterface_->getCurrentIntegratorSaturation(saturation);
}

bool Anydrive::setCurrentIntegratorSaturation(const double saturation) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the integrator saturation of the current controller.");
    return false;
  }

  if (!communicationInterface_->setCurrentIntegratorSaturation(saturation)) {
    return false;
  }
  getConfiguration().setCurrentIntegratorSaturation(saturation);
  return true;
}

bool Anydrive::getJointTorqueIntegratorSaturation(double& saturation) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the integrator saturation of the joint torque controller.");
    return false;
  }

  return communicationInterface_->getJointTorqueIntegratorSaturation(saturation);
}

bool Anydrive::setJointTorqueIntegratorSaturation(const double saturation) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the integrator saturation of the joint torque controller.");
    return false;
  }

  if (!communicationInterface_->setJointTorqueIntegratorSaturation(saturation)) {
    return false;
  }
  getConfiguration().setJointTorqueIntegratorSaturation(saturation);
  return true;
}

bool Anydrive::getDirection(int16_t& direction) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get direction.");
    return false;
  }

  return communicationInterface_->getDirection(direction);
}

bool Anydrive::setDirection(const int16_t direction) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set direction.");
    return false;
  }

  if (!communicationInterface_->setDirection(direction)) {
    return false;
  }
  getConfiguration().setDirection(direction);
  return true;
}

bool Anydrive::getJointPositionLimitsSdk(common::Limits& limits) const {
  limits = getConfiguration().getJointPositionLimitsSdk();
  return true;
}

bool Anydrive::setJointPositionLimitsSdk(const common::Limits& limits) {
  getConfiguration().setJointPositionLimitsSdk(limits);
  return true;
}

bool Anydrive::getJointPositionLimitsSoft(common::Limits& limits) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get soft joint position limits.");
    return false;
  }

  return communicationInterface_->getJointPositionLimitsSoft(limits);
}

bool Anydrive::setJointPositionLimitsSoft(const common::Limits& limits) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set soft joint position limits.");
    return false;
  }

  if (!communicationInterface_->setJointPositionLimitsSoft(limits)) {
    return false;
  }
  getConfiguration().setJointPositionLimitsSoft(limits);
  return true;
}

bool Anydrive::getJointPositionLimitsHard(common::Limits& limits) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get hard joint position limits.");
    return false;
  }

  return communicationInterface_->getJointPositionLimitsHard(limits);
}

bool Anydrive::setJointPositionLimitsHard(const common::Limits& limits) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set hard joint position limits.");
    return false;
  }

  if (!communicationInterface_->setJointPositionLimitsHard(limits)) {
    return false;
  }
  getConfiguration().setJointPositionLimitsHard(limits);
  return true;
}

bool Anydrive::getControlGains(const mode::ModeEnum mode, mode::PidGainsF& pidGains) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get control gains.");
    return false;
  }

  return communicationInterface_->getControlGains(mode, pidGains);
}

bool Anydrive::setControlGains(const mode::ModeEnum mode, const mode::PidGainsF& pidGains) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set control gains.");
    return false;
  }

  if (!communicationInterface_->setControlGains(mode, pidGains)) {
    return false;
  }
  getConfiguration().getMode(mode)->setPidGains(pidGains);
  return true;
}

bool Anydrive::getErrorStateBehavior(uint16_t& errorStateBehavior) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get error state behavior.");
    return false;
  }

  return communicationInterface_->getErrorStateBehavior(errorStateBehavior);
}

bool Anydrive::setErrorStateBehavior(const uint16_t errorStateBehavior) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set error state behavior.");
    return false;
  }

  if (!communicationInterface_->setErrorStateBehavior(errorStateBehavior)) {
    return false;
  }
  getConfiguration().setErrorStateBehavior(errorStateBehavior);
  return true;
}

bool Anydrive::getImuEnabled(bool& enabled) const {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get IMU enabled.");
    return false;
  }

  return communicationInterface_->getImuEnabled(enabled);
}

bool Anydrive::setImuEnable(const bool enable) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set IMU enable.");
    return false;
  }

  if (!communicationInterface_->setImuEnable(enable)) {
    return false;
  }
  getConfiguration().setImuEnable(enable);
  return true;
}

bool Anydrive::getImuAccelerometerRange(uint32_t& range) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get IMU accelerometer range.");
    return false;
  }

  return communicationInterface_->getImuAccelerometerRange(range);
}

bool Anydrive::setImuAccelerometerRange(const uint32_t range) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set IMU accelerometer range.");
    return false;
  }

  if (!communicationInterface_->setImuAccelerometerRange(range)) {
    return false;
  }
  getConfiguration().setImuAccelerometerRange(range);
  return true;
}

bool Anydrive::getImuGyroscopeRange(uint32_t& range) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get IMU gyroscope range.");
    return false;
  }

  return communicationInterface_->getImuGyroscopeRange(range);
}

bool Anydrive::setImuGyroscopeRange(const uint32_t range) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set IMU gyroscope range.");
    return false;
  }

  if (!communicationInterface_->setImuGyroscopeRange(range)) {
    return false;
  }
  getConfiguration().setImuGyroscopeRange(range);
  return true;
}

bool Anydrive::getFanMode(uint32_t& mode) {
  return communicationInterface_->getFanMode(mode);
}

bool Anydrive::setFanMode(const uint32_t mode) {
  if (!communicationInterface_->setFanMode(mode)) {
    return false;
  }
  getConfiguration().setFanMode(mode);
  return true;
}

bool Anydrive::getFanIntensity(uint32_t& intensity) {
  return communicationInterface_->getFanIntensity(intensity);
}

bool Anydrive::setFanIntensity(const uint32_t intensity) {
  if (!communicationInterface_->setFanIntensity(intensity)) {
    return false;
  }
  getConfiguration().setFanIntensity(intensity);
  return true;
}

bool Anydrive::getFanLowerTemperature(float& temperature) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the fan lower temperature.");
    return false;
  }

  return communicationInterface_->getFanLowerTemperature(temperature);
}

bool Anydrive::setFanLowerTemperature(const float temperature) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the fan lower temperature.");
    return false;
  }

  if (!communicationInterface_->setFanLowerTemperature(temperature)) {
    return false;
  }
  getConfiguration().setFanLowerTemperature(temperature);
  return true;
}

bool Anydrive::getFanUpperTemperature(float& temperature) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the fan upper temperature.");
    return false;
  }

  return communicationInterface_->getFanUpperTemperature(temperature);
}

bool Anydrive::setFanUpperTemperature(const float temperature) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the fan upper temperature.");
    return false;
  }

  if (!communicationInterface_->setFanUpperTemperature(temperature)) {
    return false;
  }
  getConfiguration().setFanUpperTemperature(temperature);
  return true;
}

bool Anydrive::setBrakeMode(const bool mode) {
  return communicationInterface_->setBrakeMode(mode);
}

bool Anydrive::getBrakeMode(bool& mode) {
  return communicationInterface_->getBrakeMode(mode);
}

bool Anydrive::setBrakeDuty(const float d) {
  return communicationInterface_->setBrakeDuty(d);
}

bool Anydrive::getBrakeDuty(float& d) {
  return communicationInterface_->getBrakeDuty(d);
}

bool Anydrive::getGearJointVelocityFilterType(uint32_t& type) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the gear/joint velocity filter type.");
    return false;
  }
  return communicationInterface_->getGearJointVelocityFilterType(type);
}

bool Anydrive::setGearJointVelocityFilterType(const uint32_t type) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the gear/joint velocity filter type.");
    return false;
  }

  if (!communicationInterface_->setGearJointVelocityFilterType(type)) {
    return false;
  }
  getConfiguration().setGearJointVelocityFilterType(type);
  return true;
}

bool Anydrive::getGearJointVelocityKfNoiseVariance(float& variance) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the gear/joint velocity KF noise variance.");
    return false;
  }
  return communicationInterface_->getGearJointVelocityKfNoiseVariance(variance);
}

bool Anydrive::setGearJointVelocityKfNoiseVariance(const float variance) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the gear/joint velocity KF noise variance.");
    return false;
  }

  if (!communicationInterface_->setGearJointVelocityKfNoiseVariance(variance)) {
    return false;
  }
  getConfiguration().setGearJointVelocityKfNoiseVariance(variance);
  return true;
}

bool Anydrive::getGearJointVelocityKfLambda2(float& lambda) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the gear/joint velocity KF lambda^2.");
    return false;
  }
  return communicationInterface_->getGearJointVelocityKfLambda2(lambda);
}

bool Anydrive::setGearJointVelocityKfLambda2(const float lambda) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the gear/joint velocity KF lambda^2.");
    return false;
  }

  if (!communicationInterface_->setGearJointVelocityKfLambda2(lambda)) {
    return false;
  }
  getConfiguration().setGearJointVelocityKfLambda2(lambda);
  return true;
}

bool Anydrive::getGearJointVelocityKfGamma(float& gamma) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the gear/joint velocity KF gamma.");
    return false;
  }
  return communicationInterface_->getGearJointVelocityKfGamma(gamma);
}

bool Anydrive::setGearJointVelocityKfGamma(const float gamma) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the gear/joint velocity KF gamma.");
    return false;
  }

  if (!communicationInterface_->setGearJointVelocityKfGamma(gamma)) {
    return false;
  }
  getConfiguration().setGearJointVelocityKfGamma(gamma);
  return true;
}

bool Anydrive::getGearJointVelocityEmaAlpha(float& alpha) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the gear/joint velocity EMA alpha.");
    return false;
  }
  return communicationInterface_->getGearJointVelocityEmaAlpha(alpha);
}

bool Anydrive::setGearJointVelocityEmaAlpha(const float alpha) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the gear/joint velocity EMA alpha.");
    return false;
  }

  if (!communicationInterface_->setGearJointVelocityEmaAlpha(alpha)) {
    return false;
  }
  getConfiguration().setGearJointVelocityEmaAlpha(alpha);
  return true;
}

bool Anydrive::getJointVelocityForAccelerationFilterType(uint32_t& type) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint velocity for acceleration filter type.");
    return false;
  }
  return communicationInterface_->getJointVelocityForAccelerationFilterType(type);
}

bool Anydrive::setJointVelocityForAccelerationFilterType(const uint32_t type) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint velocity for acceleration filter type.");
    return false;
  }

  if (!communicationInterface_->setJointVelocityForAccelerationFilterType(type)) {
    return false;
  }
  getConfiguration().setJointVelocityForAccelerationFilterType(type);
  return true;
}

bool Anydrive::getJointVelocityForAccelerationKfNoiseVariance(float& variance) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR(
        "Device needs to be in configure state to get the joint velocity for acceleration filter KF noise variance.");
    return false;
  }
  return communicationInterface_->getJointVelocityForAccelerationKfNoiseVariance(variance);
}

bool Anydrive::setJointVelocityForAccelerationKfNoiseVariance(const float variance) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR(
        "Device needs to be in configure state to set the joint velocity for acceleration filter KF noise variance.");
    return false;
  }

  if (!communicationInterface_->setJointVelocityForAccelerationKfNoiseVariance(variance)) {
    return false;
  }
  getConfiguration().setJointVelocityForAccelerationKfNoiseVariance(variance);
  return true;
}

bool Anydrive::getJointVelocityForAccelerationKfLambda2(float& lambda) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint velocity for acceleration filter KF lambda^2.");
    return false;
  }
  return communicationInterface_->getJointVelocityForAccelerationKfLambda2(lambda);
}

bool Anydrive::setJointVelocityForAccelerationKfLambda2(const float lambda) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint velocity for acceleration filter KF lambda^2.");
    return false;
  }

  if (!communicationInterface_->setJointVelocityForAccelerationKfLambda2(lambda)) {
    return false;
  }
  getConfiguration().setJointVelocityForAccelerationKfLambda2(lambda);
  return true;
}

bool Anydrive::getJointVelocityForAccelerationKfGamma(float& gamma) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint velocity for acceleration filter KF gamma.");
    return false;
  }
  return communicationInterface_->getJointVelocityForAccelerationKfGamma(gamma);
}

bool Anydrive::setJointVelocityForAccelerationKfGamma(const float gamma) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint velocity for acceleration filter KF gamma.");
    return false;
  }

  if (!communicationInterface_->setJointVelocityForAccelerationKfGamma(gamma)) {
    return false;
  }
  getConfiguration().setJointVelocityForAccelerationKfGamma(gamma);
  return true;
}

bool Anydrive::getJointVelocityForAccelerationEmaAlpha(float& alpha) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint velocity for acceleration EMA alpha.");
    return false;
  }
  return communicationInterface_->getJointVelocityForAccelerationEmaAlpha(alpha);
}

bool Anydrive::setJointVelocityForAccelerationEmaAlpha(const float alpha) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint velocity for acceleration EMA alpha.");
    return false;
  }

  if (!communicationInterface_->setJointVelocityForAccelerationEmaAlpha(alpha)) {
    return false;
  }
  getConfiguration().setJointVelocityForAccelerationEmaAlpha(alpha);
  return true;
}

bool Anydrive::getJointAccelerationFilterType(uint32_t& type) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint acceleration filter type.");
    return false;
  }
  return communicationInterface_->getJointAccelerationFilterType(type);
}

bool Anydrive::setJointAccelerationFilterType(const uint32_t type) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint acceleration filter type.");
    return false;
  }

  if (!communicationInterface_->setJointAccelerationFilterType(type)) {
    return false;
  }
  getConfiguration().setJointAccelerationFilterType(type);
  return true;
}

bool Anydrive::getJointAccelerationKfNoiseVariance(float& variance) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint acceleration filter KF noise variance.");
    return false;
  }
  return communicationInterface_->getJointAccelerationKfNoiseVariance(variance);
}

bool Anydrive::setJointAccelerationKfNoiseVariance(const float variance) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint acceleration filter KF noise variance.");
    return false;
  }

  if (!communicationInterface_->setJointAccelerationKfNoiseVariance(variance)) {
    return false;
  }
  getConfiguration().setJointAccelerationKfNoiseVariance(variance);
  return true;
}

bool Anydrive::getJointAccelerationKfLambda2(float& lambda) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint acceleration filter KF lambda^2.");
    return false;
  }
  return communicationInterface_->getJointAccelerationKfLambda2(lambda);
}

bool Anydrive::setJointAccelerationKfLambda2(const float lambda) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint acceleration filter KF lambda^2.");
    return false;
  }

  if (!communicationInterface_->setJointAccelerationKfLambda2(lambda)) {
    return false;
  }
  getConfiguration().setJointAccelerationKfLambda2(lambda);
  return true;
}

bool Anydrive::getJointAccelerationKfGamma(float& gamma) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint acceleration filter KF gamma.");
    return false;
  }
  return communicationInterface_->getJointAccelerationKfGamma(gamma);
}

bool Anydrive::setJointAccelerationKfGamma(const float gamma) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint acceleration filter KF gamma.");
    return false;
  }

  if (!communicationInterface_->setJointAccelerationKfGamma(gamma)) {
    return false;
  }
  getConfiguration().setJointAccelerationKfGamma(gamma);
  return true;
}

bool Anydrive::getJointAccelerationEmaAlpha(float& alpha) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to get the joint acceleration filter EMA alpha.");
    return false;
  }
  return communicationInterface_->getJointAccelerationEmaAlpha(alpha);
}

bool Anydrive::setJointAccelerationEmaAlpha(const float alpha) {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to set the joint acceleration filter EMA alpha.");
    return false;
  }

  if (!communicationInterface_->setJointAccelerationEmaAlpha(alpha)) {
    return false;
  }
  getConfiguration().setJointAccelerationEmaAlpha(alpha);
  return true;
}

bool Anydrive::writeConfiguration() {
  if (getActiveStateEnum() != fsm::StateEnum::Configure) {
    ANYDRIVE_LOGGED_NAMED_ERROR("Device needs to be in configure state to write configuration.");
    return false;
  }

  return communicationInterface_->writeConfiguration();
}

Statusword Anydrive::getStatusword() const {
  return statusword_;
}

void Anydrive::sendControlword(const uint16_t controlwordId) {
  communicationInterface_->setControlword(controlwordId);
}

void Anydrive::stageDisable() {
  Command command;
  command.setModeEnum(mode::ModeEnum::Disable);
  stageCommand(command);
}

void Anydrive::stageFreeze() {
  Command command;
  command.setModeEnum(mode::ModeEnum::Freeze);
  stageCommand(command);
}

void Anydrive::stageZeroJointTorque() {
  Command command;
  command.setModeEnum(mode::ModeEnum::JointTorque);
  stageCommand(command);
}

void Anydrive::stageCommand(const Command& command) {
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
  stagedCommand_ = command;
  commandIsStaged_ = true;
}

bool Anydrive::commandIsStaged() const {
  return commandIsStaged_;
}

bool Anydrive::waitForMode(const mode::ModeEnum& mode, double timeout, double checkingFrequency) const {
  const double timeStep = 1.0 / checkingFrequency;
  double timeSlept = 0.0;
  while (true) {
    if (timeSlept > timeout) {
      return false;
    }
    if (statusword_.getModeEnum() == mode) {
      return true;
    }
    threadSleep(timeStep);
    timeSlept += timeStep;
  }
}

ReadingExtended Anydrive::getReading() const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  return reading_;
}

void Anydrive::getReading(ReadingExtended& reading) const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  reading = reading_;
}

void Anydrive::startLog(const std::string& name) {
  logger_.startLog(name);
}

void Anydrive::stopLog() {
  logger_.stopLog();
}

Log<ReadingExtended> Anydrive::getLog(const std::string& name) const {
  return logger_.getLog(name);
}

void Anydrive::clearLogs() {
  logger_.clearLogs();
}

bool Anydrive::getRtdlEnabled(bool& enabled) {
  return communicationInterface_->getRtdlEnabled(enabled);
}

bool Anydrive::setRtdlEnable(const bool enable) {
  return communicationInterface_->setRtdlEnable(enable);
}

bool Anydrive::setRtdlCommand(const uint16_t command) {
  return communicationInterface_->setRtdlCommand(command);
}

bool Anydrive::getRtdlStatus(uint16_t& status) {
  return communicationInterface_->getRtdlStatus(status);
}

bool Anydrive::getRtdlLoggingFrequency(uint16_t& frequency) {
  return communicationInterface_->getRtdlLoggingFrequency(frequency);
}

bool Anydrive::setRtdlLoggingFrequency(const uint16_t frequency) {
  return communicationInterface_->setRtdlLoggingFrequency(frequency);
}

bool Anydrive::getRtdlStreamingFrequency(uint16_t& frequency) {
  return communicationInterface_->getRtdlStreamingFrequency(frequency);
}

bool Anydrive::setRtdlStreamingFrequency(const uint16_t frequency) {
  return communicationInterface_->setRtdlStreamingFrequency(frequency);
}

bool Anydrive::getRtdlLastTimestamp(uint64_t& timestamp) {
  return communicationInterface_->getRtdlLastTimestamp(timestamp);
}

bool Anydrive::getCalibrationTypeId(uint16_t& calibrationTypeId) {
  return communicationInterface_->getCalibrationTypeId(calibrationTypeId);
}

bool Anydrive::setCalibrationTypeId(const uint16_t calibrationTypeId) {
  return communicationInterface_->setCalibrationTypeId(calibrationTypeId);
}

bool Anydrive::getLockStatus(uint32_t& status) {
  return communicationInterface_->getLockStatus(status);
}

bool Anydrive::sendPassword(const std::string& password) {
  return communicationInterface_->sendPassword(password);
}

void Anydrive::printInfo(const std::string& prefix) const {
  std::cout << prefix << getName() << std::endl;
}

void Anydrive::requestAndSetStatusword() {
  if (!statuswordRequested_) {
    ANYDRIVE_NAMED_DEBUG("Requesting statusword over SDO ...");
    communicationInterface_->requestStatusword();
    statuswordRequested_ = true;
  } else {
    Statusword statusword;
    if (communicationInterface_->getStatusword(statusword)) {
      ANYDRIVE_NAMED_DEBUG("Received statusword over SDO.");
      setStatusword(statusword);
      statuswordRequested_ = false;
    }
  }
}

void Anydrive::setStatusword(Statusword& statusword) {
  // If the stamp has not changed, we assume it is again the same statusword.
  if (statusword.getStamp() == statusword_.getStamp()) {
    return;
  }

  // Check if statusword contains new data.
  if (statusword_.isEmpty() || statusword.getData() != statusword_.getData()) {
    ANYDRIVE_NAMED_DEBUG("Received new statusword (" << statusword << ").");
    std::vector<std::string> infos;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
    std::vector<std::string> fatals;
    statusword.getMessagesDiff(statusword_, infos, warnings, errors, fatals);
    for (const std::string& info : infos) ANYDRIVE_NAMED_INFO(info);
    for (const std::string& warning : warnings) ANYDRIVE_NAMED_WARN(warning);
    for (const std::string& error : errors) ANYDRIVE_LOGGED_NAMED_ERROR(error);
    for (const std::string& fatal : fatals) ANYDRIVE_LOGGED_NAMED_ERROR(fatal);

    if (statusword.getModeEnum() != statusword_.getModeEnum()) {
      ANYDRIVE_NAMED_DEBUG("Changed mode to '" << mode::modeEnumToName(statusword.getModeEnum()) << "'.");
    }
  }

  // Always update statusword to set new time stamp.
  statusword_ = statusword;
}

}  // namespace anydrive
