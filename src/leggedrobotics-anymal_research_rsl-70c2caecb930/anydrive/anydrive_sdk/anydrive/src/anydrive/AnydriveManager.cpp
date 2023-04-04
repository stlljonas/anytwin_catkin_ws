#include <pthread.h>

// signal handler
#include <memory>
#include <signal_handler/SignalHandler.hpp>

#include "anydrive/AnydriveManager.hpp"
#include "anydrive/Exception.hpp"
#include "anydrive/calibration/routine/FrictionEstimation.hpp"
#include "anydrive/calibration/routine/GearJointEncoderOffset.hpp"
#include "anydrive/calibration/routine/GravityCompensation.hpp"
#include "anydrive/calibration/routine/SafeJointVelocity.hpp"

namespace anydrive {

AnydriveManager::AnydriveManager(const bool standalone, const bool installSignalHandler, const double timeStep)
    : standalone_(standalone),
      installSignalHandler_(installSignalHandler),
      timeStep_(timeStep),
      shutdownRequested_(false),
      jointPositionConfigurationManager_(*this) {
  if (installSignalHandler_) {
    signal_handler::SignalHandler::bindAll(&AnydriveManager::handleSignal, this);
  }

#ifndef NDEBUG
  ANYDRIVE_WARN("CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif
}

AnydriveManager::~AnydriveManager() {
  if (installSignalHandler_) {
    signal_handler::SignalHandler::unbindAll(&AnydriveManager::handleSignal, this);
  }
}

void AnydriveManager::setProcessPriority(const int priority) {
  sched_param param{};
  param.sched_priority = priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
    ANYDRIVE_WARN("Failed to set process priority to " << priority << ".");
}

void AnydriveManager::setTimeStep(const double timeStep) {
  if (isRunning()) {
    ANYDRIVE_ERROR("The ANYdrive Manager is already running, cannot set the time step.");
    return;
  }
  timeStep_ = timeStep;
}

double AnydriveManager::getTimeStep() const {
  return timeStep_;
}

void AnydriveManager::setCommunicationManager(const communication::CommunicationManagerBasePtr& communicationManager) {
  communicationManager_ = communicationManager;
}

const communication::CommunicationManagerBasePtr& AnydriveManager::getCommunicationManager() const {
  return communicationManager_;
}

JointPositionConfigurationManager& AnydriveManager::getJointPositionConfigurationManager() {
  return jointPositionConfigurationManager_;
}

const JointPositionConfigurationManager& AnydriveManager::getJointPositionConfigurationManager() const {
  return jointPositionConfigurationManager_;
}

bool AnydriveManager::loadSetup(const std::string& setupFile) {
  // If loadSetup() without argument is called, the setupFile is empty.
  // In this case, the default setup is used.
  setup::SetupPtr setup = this->createSetup();
  if (!setupFile.empty()) {
    try {
      setup->fromFile(setupFile);
    } catch (const message_logger::log::melo_fatal& exception) {
      ANYDRIVE_ERROR("Caught an exception while loading the setup: " << exception.what());
      return false;
    }
  }
  return loadSetup(setup);
}

setup::SetupPtr AnydriveManager::createSetup() const {
  return std::make_shared<setup::Setup>();
}

bool AnydriveManager::loadSetup(const setup::SetupPtr& setup) {
  anydrives_.clear();

  for (const auto& anydriveSetup : setup->anydrives_) {
    AnydrivePtr anydrive = this->createAnydrive();
    if (!anydrive->loadSetup(anydriveSetup)) {
      return false;
    }
    if (!addAnydrive(anydrive)) {
      return false;
    }
  }

  if (!communicationManager_->loadSetup(setup, this)) {
    return false;
  }

  if (!jointPositionConfigurationManager_.loadSetup(setup->jointPositionConfigurationManager_)) {
    return false;
  }

  return true;
}

bool AnydriveManager::addAnydrive(const AnydrivePtr& anydrive) {
  const std::string name = anydrive->getName();
  if (anydriveExists(name)) {
    ANYDRIVE_ERROR("Cannot add ANYdrive with name '" << name << "', because it already exists.");
    return false;
  }
  anydrives_.push_back(anydrive);
  return true;
}

bool AnydriveManager::anydriveExists(const std::string& name) const {
  for (const auto& anydrive : anydrives_) {
    if (anydrive->getName() == name) {
      return true;
    }
  }
  return false;
}

AnydrivePtr AnydriveManager::getAnydrive(const std::string& name) const {
  for (auto& anydrive : anydrives_) {
    if (anydrive->getName() == name) {
      return anydrive;
    }
  }
  return AnydrivePtr();
}

AnydriveManager::Anydrives AnydriveManager::getAnydrives() const {
  return anydrives_;
}

unsigned int AnydriveManager::getNumberOfAnydrives() const {
  return anydrives_.size();
}

void AnydriveManager::addReadingCb(const ReadingCb& readingCb, const int priority) {
  for (const auto& anydrive : anydrives_) {
    anydrive->addReadingCb(readingCb, priority);
  }
}

void AnydriveManager::addErrorCb(const ErrorCb& errorCb, const int priority) {
  for (const auto& anydrive : anydrives_) {
    anydrive->addErrorCb(errorCb, priority);
  }
}

void AnydriveManager::addErrorRecoveredCb(const ErrorRecoveredCb& errorRecoveredCb, const int priority) {
  for (const auto& anydrive : anydrives_) {
    anydrive->addErrorRecoveredCb(errorRecoveredCb, priority);
  }
}

void AnydriveManager::addFatalCb(const FatalCb& fatalCb, const int priority) {
  for (const auto& anydrive : anydrives_) {
    anydrive->addFatalCb(fatalCb, priority);
  }
}

void AnydriveManager::addFatalRecoveredCb(const FatalRecoveredCb& fatalRecoveredCb, const int priority) {
  for (const auto& anydrive : anydrives_) {
    anydrive->addFatalRecoveredCb(fatalRecoveredCb, priority);
  }
}

void AnydriveManager::addDeviceDisconnectedCb(const DeviceDisconnectedCb& deviceDisconnectedCb, const int priority) {
  for (const auto& anydrive : anydrives_) {
    anydrive->addDeviceDisconnectedCb(deviceDisconnectedCb, priority);
  }
}

void AnydriveManager::addDeviceReconnectedCb(const DeviceReconnectedCb& deviceReconnectedCb, const int priority) {
  for (const auto& anydrive : anydrives_) {
    anydrive->addDeviceReconnectedCb(deviceReconnectedCb, priority);
  }
}

bool AnydriveManager::startup() {
  return startup(1);
}

bool AnydriveManager::startup(unsigned int startupRetries) {
  ANYDRIVE_DEBUG("Waiting for mutex to be free ...");
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  ANYDRIVE_DEBUG("Locked mutex.");
  if (isRunning_) {
    ANYDRIVE_WARN("Cannot start up, ANYdrive Manager is already running.");
    return false;
  }

  ANYDRIVE_INFO("Starting up ANYdrive Manager ...");

  if (!communicationManager_) {
    throw Exception("Cannot start up, communication manager has not been set.");
  }

  for (const auto& anydrive : anydrives_) {
    anydrive->startupWithoutCommunication();
  }
  unsigned int countStartupTries = 0;
  while (!communicationManager_->startup()) {
    ++countStartupTries;
    if ((startupRetries > 0 && countStartupTries == startupRetries) || shutdownRequested_) {
      for (const auto& anydrive : anydrives_) {
        anydrive->shutdownWithoutCommunication();
      }
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ANYDRIVE_INFO("Try to startup again.")
  }
  if (standalone_) {
    any_worker::WorkerOptions updateWorkerOptions;
    updateWorkerOptions.callback_ = std::bind(&AnydriveManager::updateWorkerCb, this, std::placeholders::_1);
    updateWorkerOptions.defaultPriority_ = 90;
    updateWorkerOptions.name_ = "AnydriveManager::updateWorker";
    updateWorkerOptions.timeStep_ = timeStep_;
    updateWorkerOptions.enforceRate_ = true;
    updateWorker_.reset(new any_worker::Worker(updateWorkerOptions));
    if (!updateWorker_->start()) {
      throw Exception("Update worker could not be started.");
    }
  }
  for (const auto& anydrive : anydrives_) {
    anydrive->startupWithCommunication();
  }

  isRunning_ = true;
  ANYDRIVE_DEBUG("Started up ANYdrive Manager.");
  return true;
}

bool AnydriveManager::update() {
  updateCommunicationManagerReadMessages();
  updateProcessReadings();
  updateStageCommands();
  updateSendStagedCommands();
  updateCommunicationManagerWriteMessages();
  return true;
}

void AnydriveManager::updateCommunicationManagerReadMessages() {
  communicationManager_->updateRead();
}

void AnydriveManager::updateProcessReadings() {
  for (const auto& anydrive : anydrives_) {
    anydrive->updateProcessReading();
  }
}

void AnydriveManager::updateStageCommands() {
  std::vector<Command> commands;
  if (jointPositionConfigurationManager_.getNextCommandsInQueue(commands)) {
    stageCommands(commands);
  }
}

void AnydriveManager::updateSendStagedCommands() {
  for (const auto& anydrive : anydrives_) {
    anydrive->updateSendStagedCommand();
  }
}

void AnydriveManager::updateCommunicationManagerWriteMessages() {
  communicationManager_->updateWrite();
}

void AnydriveManager::shutdown() {
  ANYDRIVE_DEBUG("Waiting for mutex to be free ...");
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  ANYDRIVE_DEBUG("Locked mutex.");
  if (!isRunning_) {
    ANYDRIVE_WARN("Cannot shut down, ANYdrive Manager is not running.");
    return;
  }

  ANYDRIVE_INFO("Shutting down ANYdrive Manager ...");

  if (runningCalibration_) {
    runningCalibration_->shutdown();
  }

  ANYDRIVE_DEBUG("Preparing ANYdrives shutdown ...");
  // If not in standalone mode, the exterior worker is already shutdown at this point.
  // For this reason, a temporary worker has to take over the job.
  if (!standalone_) {
    any_worker::WorkerOptions updateWorkerOptions;
    updateWorkerOptions.callback_ = std::bind(&AnydriveManager::updateWorkerCb, this, std::placeholders::_1);
    updateWorkerOptions.defaultPriority_ = 90;
    updateWorkerOptions.name_ = "AnydriveManager::updateWorker (shutdown)";
    updateWorkerOptions.timeStep_ = timeStep_;
    updateWorkerOptions.enforceRate_ = true;
    updateWorker_.reset(new any_worker::Worker(updateWorkerOptions));
    if (!updateWorker_->start()) ANYDRIVE_ERROR("Update worker (shutdown) could not be started.");
  }
  for (const auto& anydrive : anydrives_) {
    anydrive->shutdownWithCommunication(isCommunicationOk(), 5.0, 100);
  }
  // Stop the worker here (in both modes, standalone and not).
  updateWorker_->stop(true);
  ANYDRIVE_DEBUG("Shutting down communication manager ...");
  communicationManager_->shutdown();
  ANYDRIVE_DEBUG("Shutting down ANYdrives ...");
  for (const auto& anydrive : anydrives_) {
    anydrive->shutdownWithoutCommunication();
  }

  isRunning_ = false;
  ANYDRIVE_DEBUG("Shut down ANYdrive Manager.");
}

bool AnydriveManager::isRunning() const {
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  return isRunning_;
}

bool AnydriveManager::shutdownRequested() const {
  return shutdownRequested_;
}

bool AnydriveManager::setGoalStatesEnum(const fsm::StateEnum goalStateEnum, const bool reachStates, const double timeout,
                                        const double checkingFrequency) {
  for (const auto& anydrive : anydrives_) {
    anydrive->setGoalStateEnum(goalStateEnum);
  }

  if (!reachStates) {
    return true;
  }

  const double timeStep = 1.0 / checkingFrequency;
  double timeSlept = 0.0;
  while (true) {
    if (timeSlept > timeout) {
      return false;
    }
    bool goalStatesHaveBeenReached = true;
    for (const auto& anydrive : anydrives_) {
      goalStatesHaveBeenReached = goalStatesHaveBeenReached && anydrive->goalStateHasBeenReached();
    }
    if (goalStatesHaveBeenReached) {
      return true;
    }
    threadSleep(timeStep);
    timeSlept += timeStep;
  }
}

bool AnydriveManager::isCommunicationOk() const {
  return communicationManager_->isCommunicationOk();
}

unsigned int AnydriveManager::getWorkingCounterTooLowCount() const {
  return communicationManager_->getWorkingCounterTooLowCount();
}

void AnydriveManager::clearGoalStatesEnum() {
  for (const auto& anydrive : anydrives_) {
    anydrive->clearGoalStateEnum();
  }
}

bool AnydriveManager::allDevicesAreConnected() const {
  for (const auto& anydrive : anydrives_) {
    if (anydrive->deviceIsMissing()) {
      return false;
    }
  }
  return isCommunicationOk();
}

bool AnydriveManager::allDevicesAreInTheState(const fsm::StateEnum stateEnum) const {
  if (getNumberOfAnydrives() == 0) {
    return false;
  }

  for (const auto& anydrive : anydrives_) {
    if (anydrive->getActiveStateEnum() != stateEnum) {
      return false;
    }
  }
  return true;
}

bool AnydriveManager::allDevicesAreInTheSameState(fsm::StateEnum& stateEnum) const {
  if (getNumberOfAnydrives() == 0) {
    stateEnum = fsm::StateEnum::NA;
    return true;
  }

  bool stateEnumSet = false;
  for (const auto& anydrive : anydrives_) {
    if (!stateEnumSet) {
      stateEnum = anydrive->getActiveStateEnum();
      stateEnumSet = true;
    } else {
      if (stateEnum != anydrive->getActiveStateEnum()) {
        stateEnum = fsm::StateEnum::NA;
        return false;
      }
    }
  }
  return true;
}

bool AnydriveManager::allDevicesAreInTheMode(const mode::ModeEnum modeEnum) const {
  if (getNumberOfAnydrives() == 0) {
    return false;
  }

  for (const auto& anydrive : anydrives_) {
    if (anydrive->getStatusword().getModeEnum() != modeEnum) {
      return false;
    }
  }
  return true;
}

bool AnydriveManager::allDevicesAreInTheSameMode(mode::ModeEnum& modeEnum) const {
  if (getNumberOfAnydrives() == 0) {
    modeEnum = mode::ModeEnum::NA;
    return true;
  }

  bool modeEnumSet = false;
  for (const auto& anydrive : anydrives_) {
    if (!modeEnumSet) {
      modeEnum = anydrive->getStatusword().getModeEnum();
      modeEnumSet = true;
    } else {
      if (modeEnum != anydrive->getStatusword().getModeEnum()) {
        modeEnum = mode::ModeEnum::NA;
        return false;
      }
    }
  }
  return true;
}

bool AnydriveManager::noDeviceIsInErrorState() const {
  for (const auto& anydrive : anydrives_) {
    if (anydrive->deviceIsInErrorState()) {
      return false;
    }
  }
  return true;
}

bool AnydriveManager::noDeviceIsInFatalState() const {
  for (const auto& anydrive : anydrives_) {
    if (anydrive->deviceIsInFatalState()) {
      return false;
    }
  }
  return true;
}

bool AnydriveManager::allDevicesAreWithinJointPositionLimitsSdk() const {
  for (const auto& anydrive : anydrives_) {
    if (!anydrive->isWithinJointPositionLimitsSdk()) {
      return false;
    }
  }
  return true;
}

bool AnydriveManager::allDevicesAreWithinJointPositionLimitsSoft() const {
  for (const auto& anydrive : anydrives_) {
    if (!anydrive->isWithinJointPositionLimitsSoft()) {
      return false;
    }
  }
  return true;
}

bool AnydriveManager::allDevicesAreWithinJointPositionLimitsHard() const {
  for (const auto& anydrive : anydrives_) {
    if (!anydrive->isWithinJointPositionLimitsHard()) {
      return false;
    }
  }
  return true;
}

bool AnydriveManager::eraseFlashStorage() {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->eraseFlashStorage();
  }
  return success;
}

bool AnydriveManager::resetFlashStorageSections(const uint8_t flashStorageSections) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->resetFlashStorageSections(flashStorageSections);
  }
  return success;
}

bool AnydriveManager::calibrate(const std::string& /*deviceName*/, const calibration::CalibrationModeEnum /*calibrationModeEnum*/,
                                const bool /*gearAndJointEncoderHomingAbsolute*/,
                                const double /*gearAndJointEncoderHomingNewJointPosition*/) {
  ANYDRIVE_WARN("[AnydriveManager::calibrate] is not implemented");
  return false;
}

bool AnydriveManager::resetCustomCalibrationsToFactory(const calibration::CalibrationState calibrationState) {
  bool success = true;
  for (auto& anydrive : anydrives_) {
    success &= anydrive->resetCustomCalibrationsToFactory(calibrationState);
  }
  return success;
}

bool AnydriveManager::writeFactoryCalibration() {
  bool success = true;
  for (auto& anydrive : anydrives_) {
    success &= anydrive->writeFactoryCalibration();
  }
  return success;
}

bool AnydriveManager::setMaxCurrent(const double maxCurrent) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setMaxCurrent(maxCurrent);
  }
  return success;
}

bool AnydriveManager::setMaxMotorVelocity(const double maxMotorVelocity) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setMaxMotorVelocity(maxMotorVelocity);
  }
  return success;
}

bool AnydriveManager::setMaxJointTorque(const double maxJointTorque) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setMaxJointTorque(maxJointTorque);
  }
  return success;
}

bool AnydriveManager::setCurrentIntegratorSaturation(const double saturation) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setCurrentIntegratorSaturation(saturation);
  }
  return success;
}

bool AnydriveManager::setJointTorqueIntegratorSaturation(const double saturation) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setJointTorqueIntegratorSaturation(saturation);
  }
  return success;
}

bool AnydriveManager::setDirection(const int16_t direction) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setDirection(direction);
  }
  return success;
}

bool AnydriveManager::setJointPositionLimitsSdk(const common::Limits& limits) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setJointPositionLimitsSdk(limits);
  }
  return success;
}

bool AnydriveManager::setJointPositionLimitsSoft(const common::Limits& limits) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setJointPositionLimitsSoft(limits);
  }
  return success;
}

bool AnydriveManager::setJointPositionLimitsHard(const common::Limits& limits) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setJointPositionLimitsHard(limits);
  }
  return success;
}

bool AnydriveManager::setControlGains(mode::ModeEnum mode, const mode::PidGainsF& pidGains) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setControlGains(mode, pidGains);
  }
  return success;
}

bool AnydriveManager::setErrorStateBehavior(const uint16_t errorStateBehavior) {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->setErrorStateBehavior(errorStateBehavior);
  }
  return success;
}

bool AnydriveManager::writeConfiguration() {
  bool success = true;
  for (const auto& anydrive : anydrives_) {
    success &= anydrive->writeConfiguration();
  }
  return success;
}

void AnydriveManager::sendControlwords(const uint16_t controlwordId) {
  for (const auto& anydrive : anydrives_) {
    anydrive->sendControlword(controlwordId);
  }
}

void AnydriveManager::stageDisables() {
  for (const auto& anydrive : anydrives_) {
    anydrive->stageDisable();
  }
}

void AnydriveManager::stageFreezes() {
  for (const auto& anydrive : anydrives_) {
    anydrive->stageFreeze();
  }
}

void AnydriveManager::stageZeroJointTorques() {
  for (const auto& anydrive : anydrives_) {
    anydrive->stageZeroJointTorque();
  }
}

void AnydriveManager::stageCommands(const std::vector<Command>& commands) {
  ANYDRIVE_ASSERT(commands.size() == anydrives_.size(), "The number of commands does not match the number of ANYdrives.");
  unsigned int i = 0;
  for (const auto& anydrive : anydrives_) {
    anydrive->stageCommand(commands[i]);
    i++;
  }
}

bool AnydriveManager::achieveJointPositionConfiguration(const std::string& configurationName) {
  return jointPositionConfigurationManager_.generateCommandsQueueForConfiguration(configurationName);
}

bool AnydriveManager::achieveJointPositionConfigurationIsActive() {
  return jointPositionConfigurationManager_.isActive();
}

void AnydriveManager::abortAchieveJointPositionConfiguration() {
  jointPositionConfigurationManager_.clearCommandsInQueue();
  stageFreezes();
}

std::vector<ReadingExtended> AnydriveManager::getReadings() const {
  std::vector<ReadingExtended> readings;
  readings.reserve(anydrives_.size());
  for (const auto& anydrive : anydrives_) {
    readings.emplace_back(anydrive->getReading());
  }
  return readings;
}

void AnydriveManager::getReadings(std::vector<ReadingExtended>& readings) const {
  readings = getReadings();
}

void AnydriveManager::printInfo() const {
  std::cout << "ANYdrives:" << std::endl;
  for (const auto& anydrive : anydrives_) {
    anydrive->printInfo(" - ");
  }
}

AnydrivePtr AnydriveManager::createAnydrive() const {
  return std::make_shared<Anydrive>();
}

bool AnydriveManager::updateWorkerCb(const any_worker::WorkerEvent& /*event*/) {
  return update();
}

void AnydriveManager::requestShutdown() {
  shutdownRequested_ = true;
  if (runningCalibration_) {
    runningCalibration_->requestShutdown();
  }
}

void AnydriveManager::handleSignal(const int signum) {
  ANYDRIVE_INFO("Received signal (" << signum << "), requesting shutdown ...");
  requestShutdown();
  if (signum == SIGSEGV) {
    signal(signum, SIG_DFL);
    kill(getpid(), signum);
  }
}

}  // namespace anydrive
