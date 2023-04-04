#include <any_measurements_ros/ConvertRosMessages.hpp>

#include <anydrive/Exception.hpp>

#include "anydrive_ros/AnydriveManagerRos.hpp"
#include "anydrive_ros/conversions.hpp"

namespace anydrive_ros {

AnydriveManagerRos::AnydriveManagerRos(const bool standalone, const bool installSignalHandler, const double timeStep, ros::NodeHandle& nh,
                                       std::string rosPrefix)
    : AnydriveManager(standalone, installSignalHandler, timeStep),
      nh_(nh),
      rosPrefix_(std::move(rosPrefix)),
      readingsMsgUpdated_(false),
      readingsExtendedMsgUpdated_(false),
      jointStatesMsgUpdated_(false),
      publishCounter_(0),
      shutdownPublishWorkerRequested_(false) {}

anydrive_msgs::Readings AnydriveManagerRos::getReadingsMsg() {
  std::lock_guard<std::recursive_mutex> lock(readingsMsgMutex_);
  if (!readingsMsgUpdated_) {
    unsigned int i = 0;
    for (const auto& anydrive : getAnydrives()) {
      readingsMsg_.readings[i] = std::static_pointer_cast<AnydriveRos>(anydrive)->getReadingMsg();
      i++;
    }
    readingsMsgUpdated_ = true;
  }
  return readingsMsg_;
}

anydrive_msgs::ReadingsExtended AnydriveManagerRos::getReadingsExtendedMsg() {
  std::lock_guard<std::recursive_mutex> lock(readingsExtendedMsgMutex_);
  if (!readingsExtendedMsgUpdated_) {
    unsigned int i = 0;
    for (const auto& anydrive : getAnydrives()) {
      readingsExtendedMsg_.readings[i] = std::static_pointer_cast<AnydriveRos>(anydrive)->getReadingExtendedMsg();
      i++;
    }
    readingsExtendedMsgUpdated_ = true;
  }
  return readingsExtendedMsg_;
}

sensor_msgs::JointState AnydriveManagerRos::getJointStatesMsg() {
  std::lock_guard<std::recursive_mutex> lock(jointStatesMsgMutex_);
  if (!jointStatesMsgUpdated_) {
    unsigned int i = 0;
    for (const auto& anydrive : getAnydrives()) {
      const anydrive::StateExtended state = anydrive->getReading().getState();
      if (i == 0) {
        jointStatesMsg_.header.stamp = any_measurements_ros::toRos(state.getStamp());
      }
      jointStatesMsg_.name[i] = anydrive->getName();
      jointStatesMsg_.position[i] = state.getJointPosition();
      jointStatesMsg_.velocity[i] = state.getJointVelocity();
      jointStatesMsg_.effort[i] = state.getJointTorque();
      i++;
    }
    jointStatesMsgUpdated_ = true;
  }
  return jointStatesMsg_;
}

bool AnydriveManagerRos::startup() {
  return startup(1);
}

bool AnydriveManagerRos::startup(unsigned int startupRetries) {
  ANYDRIVE_DEBUG("Waiting for mutex to be free ...");
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  ANYDRIVE_DEBUG("Locked mutex.");
  if (isRunning_) {
    ANYDRIVE_DEBUG("Cannot start up, ANYdrive Manager is already running.");
    return false;
  }
  AnydriveManagerRos::startupRosInterface();
  if (!AnydriveManager::startup(startupRetries)) {
    AnydriveManagerRos::shutdownRosInterface();
    return false;
  }
  isRunning_ = true;
  return true;
}

void AnydriveManagerRos::updateProcessReadings() {
  AnydriveManager::updateProcessReadings();

  // Reset the messages.
  readingsMsgUpdated_ = false;
  readingsExtendedMsgUpdated_ = false;
  jointStatesMsgUpdated_ = false;

  // Publish readings.
  if (runReadingsPublisher_) {
    readingsPublisher_->publish(getReadingsMsg());
  }

  // Publish extended readings.
  if (runReadingsExtendedThrottledPublisher_ &&
      readingsExtendedThrottledPublisherCounter_ == readingsExtendedThrottledPublisherDecimation_) {
    readingsExtendedThrottledPublisher_->publish(getReadingsExtendedMsg());
    readingsExtendedThrottledPublisherCounter_ = 0;
  }
  readingsExtendedThrottledPublisherCounter_++;

  // Publish joint states.
  if (runJointStatesPublisher_) {
    jointStatesPublisher_->publish(getJointStatesMsg());
  }
  if (runJointStatesThrottledPublisher_ && jointStatesThrottledPublisherCounter_ == jointStatesThrottledPublisherDecimation_) {
    jointStatesThrottledPublisher_->publish(getJointStatesMsg());
    jointStatesThrottledPublisherCounter_ = 0;
  }
  jointStatesThrottledPublisherCounter_++;

  std::unique_lock<std::mutex> lock(notifyPublishWorkerMutex_);
  publishCounter_++;
  notifyPublishWorkerCv_.notify_all();
}

void AnydriveManagerRos::shutdown() {
  ANYDRIVE_INFO("Waiting for mutex to be free ...");
  std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  ANYDRIVE_INFO("Locked mutex.");
  if (!isRunning_) {
    ANYDRIVE_DEBUG("Cannot shut down, ANYdrive Manager is not running.");
    return;
  }
  AnydriveManager::shutdown();
  AnydriveManagerRos::shutdownRosInterface();
  isRunning_ = false;
}

void AnydriveManagerRos::sendRos() {
  if (runReadingsPublisher_) {
    readingsPublisher_->sendRos();
  }

  if (runReadingsExtendedThrottledPublisher_) {
    readingsExtendedThrottledPublisher_->sendRos();
  }

  if (runJointStatesPublisher_) {
    jointStatesPublisher_->sendRos();
  }

  if (runJointStatesThrottledPublisher_) {
    jointStatesThrottledPublisher_->sendRos();
  }

  for (const auto& anydrive : getAnydrives()) {
    std::static_pointer_cast<AnydriveRos>(anydrive)->sendRos();
  }
}

anydrive::AnydrivePtr AnydriveManagerRos::createAnydrive() const {
  return anydrive::AnydrivePtr(new AnydriveRos(nh_, maxPublishMessageBufferSize_));
}

void AnydriveManagerRos::startupRosInterface() {
  param_io::getParam(nh_, "ros_prefix", rosPrefix_);
  param_io::getParam(nh_, "subscribers/commands/enable", runCommandsSubscriber_);
  param_io::getParam(nh_, "publishers/create_worker", createPublishWorker_);
  param_io::getParam(nh_, "publishers/readings/enable", runReadingsPublisher_);
  param_io::getParam(nh_, "publishers/readings_extended_throttled/enable", runReadingsExtendedThrottledPublisher_);
  param_io::getParam(nh_, "publishers/readings_extended_throttled/decimation", readingsExtendedThrottledPublisherDecimation_);
  param_io::getParam(nh_, "publishers/joint_states/enable", runJointStatesPublisher_);
  param_io::getParam(nh_, "publishers/joint_states_throttled/enable", runJointStatesThrottledPublisher_);
  param_io::getParam(nh_, "publishers/joint_states_throttled/decimation", jointStatesThrottledPublisherDecimation_);

  if (runCommandsSubscriber_) {
    commandsSubscriber_ =
        nh_.subscribe(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "subscribers/commands/topic", "commands"),
                      param_io::param<uint32_t>(nh_, "subscribers/commands/queue_size", 1), &AnydriveManagerRos::commandsCb, this);
  }

  if (runReadingsPublisher_) {
    readingsPublisher_.reset(new any_node::ThreadedPublisher<anydrive_msgs::Readings>(
        nh_.advertise<anydrive_msgs::Readings>(
            rosPrefix_ + "/" + param_io::param<std::string>(nh_, "publishers/readings/topic", "readings"),
            param_io::param<uint32_t>(nh_, "publishers/readings/queue_size", 1),
            param_io::param<bool>(nh_, "publishers/readings/latch", false)),
        maxPublishMessageBufferSize_, false));
  }

  if (runReadingsExtendedThrottledPublisher_) {
    readingsExtendedThrottledPublisher_.reset(new any_node::ThreadedPublisher<anydrive_msgs::ReadingsExtended>(
        nh_.advertise<anydrive_msgs::ReadingsExtended>(
            rosPrefix_ + "/" +
                param_io::param<std::string>(nh_, "publishers/readings_extended_throttled/topic", "readings_extended_throttled"),
            param_io::param<uint32_t>(nh_, "publishers/readings_extended_throttled/queue_size", 1),
            param_io::param<bool>(nh_, "publishers/readings_extended_throttled/latch", false)),
        maxPublishMessageBufferSize_, false));
  }

  if (runJointStatesPublisher_) {
    jointStatesPublisher_.reset(new any_node::ThreadedPublisher<sensor_msgs::JointState>(
        nh_.advertise<sensor_msgs::JointState>(
            rosPrefix_ + "/" + param_io::param<std::string>(nh_, "publishers/joint_states/topic", "joint_states"),
            param_io::param<uint32_t>(nh_, "publishers/joint_states/queue_size", 1),
            param_io::param<bool>(nh_, "publishers/joint_states/latch", false)),
        maxPublishMessageBufferSize_, false));
  }

  if (runJointStatesThrottledPublisher_) {
    jointStatesThrottledPublisher_.reset(new any_node::ThreadedPublisher<sensor_msgs::JointState>(
        nh_.advertise<sensor_msgs::JointState>(
            rosPrefix_ + "/" + param_io::param<std::string>(nh_, "publishers/joint_states_throttled/topic", "joint_states_throttled"),
            param_io::param<uint32_t>(nh_, "publishers/joint_states_throttled/queue_size", 1),
            param_io::param<bool>(nh_, "publishers/joint_states_throttled/latch", false)),
        maxPublishMessageBufferSize_, false));
  }

  // No threaded publisher here, since we publish only once.
  availableJointPositionConfigurationsPublisher_ = nh_.advertise<anydrive_msgs::JointPositionConfigurations>(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "publishers/available_joint_position_configurations/topic",
                                       "available_joint_position_configurations"),
      param_io::param<uint32_t>(nh_, "publishers/available_joint_position_configurations/queue_size", 1),
      param_io::param<bool>(nh_, "publishers/available_joint_position_configurations/latch", false));

  // Send SDOs.
  sendSdoReadServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/send_sdo_read/service", "send_sdo_read"),
                           &AnydriveManagerRos::sendSdoReadCb, this);
  sendSdoWriteServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/send_sdo_write/service", "send_sdo_write"),
                           &AnydriveManagerRos::sendSdoWriteCb, this);

  // Drive info.
  getDriveInfoServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/get_drive_info/service", "get_drive_info"),
                           &AnydriveManagerRos::getDriveInfoCb, this);
  setDriveInfoSerialNumberServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_drive_info_serial_number/service", "set_drive_info_serial_number"),
      &AnydriveManagerRos::setDriveInfoSerialNumber, this);
  setDriveInfoNameServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_drive_info_name/service", "set_drive_info_name"),
      &AnydriveManagerRos::setDriveInfoName, this);
  setDriveInfoIdServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_drive_info_id/service", "set_drive_info_id"),
                           &AnydriveManagerRos::setDriveInfoId, this);
  setDriveInfoBootloaderVersionServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/set_drive_info_bootloader_version/service", "set_drive_info_bootloader_version"),
      &AnydriveManagerRos::setDriveInfoBootloaderVersion, this);

  // Flash storage.
  eraseFlashStorage_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/erase_flash_storage/service", "erase_flash_storage"),
      &AnydriveManagerRos::eraseFlashStorageCb, this);
  resetFlashStorageSections_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/reset_flash_storage_sections/service", "reset_flash_storage_sections"),
      &AnydriveManagerRos::resetFlashStorageSectionsCb, this);

  // Calibration.
  getCalibrationStateServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/get_calibration_state/service", "get_calibration_state"),
      &AnydriveManagerRos::getCalibrationStateCb, this);
  calibrateServer_ = nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/calibrate/service", "calibrate"),
                                          &AnydriveManagerRos::calibrateCb, this);
  resetCustomCalibrationsToFactoryServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/reset_custom_calibrations_to_factory/service", "reset_custom_calibrations_to_factory"),
      &AnydriveManagerRos::resetCustomCalibrationsToFactoryCb, this);
  writeFactoryCalibrationServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/write_factory_calibration/service", "write_factory_calibration"),
      &AnydriveManagerRos::writeFactoryCalibrationCb, this);

  // Configuration.
  getMaxCurrentServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/get_max_current/service", "get_max_current"),
                           &AnydriveManagerRos::getMaxCurrentCb, this);
  setMaxCurrentServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_max_current/service", "set_max_current"),
                           &AnydriveManagerRos::setMaxCurrentCb, this);
  getMaxMotorVelocityServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/get_max_motor_velocity/service", "get_max_motor_velocity"),
      &AnydriveManagerRos::getMaxMotorVelocityCb, this);
  setMaxMotorVelocityServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_max_motor_velocity/service", "set_max_motor_velocity"),
      &AnydriveManagerRos::setMaxMotorVelocityCb, this);
  getJointPositionLimitsSdkServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/get_joint_position_limits_sdk/service", "get_joint_position_limits_sdk"),
      &AnydriveManagerRos::getJointPositionLimitsSdkCb, this);
  setJointPositionLimitsSdkServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/set_joint_position_limits_sdk/service", "set_joint_position_limits_sdk"),
      &AnydriveManagerRos::setJointPositionLimitsSdkCb, this);
  getJointPositionLimitsSoftServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/get_joint_position_limits_soft/service", "get_joint_position_limits_soft"),
      &AnydriveManagerRos::getJointPositionLimitsSoftCb, this);
  setJointPositionLimitsSoftServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/set_joint_position_limits_soft/service", "set_joint_position_limits_soft"),
      &AnydriveManagerRos::setJointPositionLimitsSoftCb, this);
  getJointPositionLimitsHardServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/get_joint_position_limits_hard/service", "get_joint_position_limits_hard"),
      &AnydriveManagerRos::getJointPositionLimitsHardCb, this);
  setJointPositionLimitsHardServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/set_joint_position_limits_hard/service", "set_joint_position_limits_hard"),
      &AnydriveManagerRos::setJointPositionLimitsHardCb, this);
  getControlGainsServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/get_control_gains/service", "get_control_gains"),
                           &AnydriveManagerRos::getControlGainsCb, this);
  setControlGainsServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_control_gains/service", "set_control_gains"),
                           &AnydriveManagerRos::setControlGainsCb, this);
  getErrorStateBehaviorServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/get_error_state_behavior/service", "get_error_state_behavior"),
      &AnydriveManagerRos::getErrorStateBehaviorCb, this);
  setErrorStateBehaviorServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_error_state_behavior/service", "set_error_state_behavior"),
      &AnydriveManagerRos::setErrorStateBehaviorCb, this);
  writeConfigurationServer_ = nh_.advertiseService(
      rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/write_configuration/service", "write_configuration"),
      &AnydriveManagerRos::writeConfigurationCb, this);

  // Control.
  setGoalStateServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/set_goal_state/service", "set_goal_state"),
                           &AnydriveManagerRos::setGoalStateCb, this);
  sendControlwordServer_ =
      nh_.advertiseService(rosPrefix_ + "/" + param_io::param<std::string>(nh_, "servers/send_controlword/service", "send_controlword"),
                           &AnydriveManagerRos::sendControlwordCb, this);
  achieveJointPositionConfigurationServer_ = nh_.advertiseService(
      rosPrefix_ + "/" +
          param_io::param<std::string>(nh_, "servers/achieve_joint_position_configuration/service", "achieve_joint_position_configuration"),
      &AnydriveManagerRos::achieveJointPositionConfigurationCb, this);

  const unsigned int numberOfAnydrives = getNumberOfAnydrives();
  {
    std::lock_guard<std::recursive_mutex> lock(readingsMsgMutex_);
    readingsMsg_.readings.resize(numberOfAnydrives);
  }
  {
    std::lock_guard<std::recursive_mutex> lock(readingsExtendedMsgMutex_);
    readingsExtendedMsg_.readings.resize(numberOfAnydrives);
  }
  {
    std::lock_guard<std::recursive_mutex> lock(jointStatesMsgMutex_);
    jointStatesMsg_.name.resize(numberOfAnydrives);
    jointStatesMsg_.position.resize(numberOfAnydrives);
    jointStatesMsg_.velocity.resize(numberOfAnydrives);
    jointStatesMsg_.effort.resize(numberOfAnydrives);
  }

  anydrive_msgs::JointPositionConfigurations configurations;
  for (const std::string& configurationName : jointPositionConfigurationManager_.getConfigurationNames()) {
    anydrive_msgs::JointPositionConfiguration configuration;
    configuration.name = configurationName;
    configurations.joint_position_configurations.push_back(configuration);
  }
  availableJointPositionConfigurationsPublisher_.publish(configurations);

  // If required, start a publish worker(after the publishers have been created).
  if (createPublishWorker_ &&
      (runReadingsPublisher_ || runReadingsExtendedThrottledPublisher_ || runJointStatesPublisher_ || runJointStatesThrottledPublisher_)) {
    any_worker::WorkerOptions publishWorkerOptions;
    publishWorkerOptions.callback_ = std::bind(&AnydriveManagerRos::publishWorkerCb, this, std::placeholders::_1);
    publishWorkerOptions.defaultPriority_ = 10;
    publishWorkerOptions.name_ = "AnydriveManagerRos::publishWorker";
    publishWorkerOptions.timeStep_ =
        std::numeric_limits<double>::infinity();  // Run only once, similar to creating a thread but with priority.
    publishWorker_.reset(new any_worker::Worker(publishWorkerOptions));
    if (!publishWorker_->start()) {
      throw anydrive::Exception("Publish worker could not be started.");
    }
  }
}

void AnydriveManagerRos::shutdownRosInterface() {
  ANYDRIVE_INFO("Shutting down ANYdrive Manager ROS interface ...");

  // Shut down the publish worker (before the publishers are shut down).
  shutdownPublishWorkerRequested_ = true;
  notifyPublishWorkerCv_.notify_all();
  if (publishWorker_ != nullptr) {
    publishWorker_->stop();
  }

  if (runCommandsSubscriber_) {
    commandsSubscriber_.shutdown();
  }

  if (runReadingsPublisher_) {
    readingsPublisher_->shutdown();
  }
  if (runReadingsExtendedThrottledPublisher_) {
    readingsExtendedThrottledPublisher_->shutdown();
  }
  if (runJointStatesPublisher_) {
    jointStatesPublisher_->shutdown();
  }
  if (runJointStatesThrottledPublisher_) {
    jointStatesThrottledPublisher_->shutdown();
  }

  availableJointPositionConfigurationsPublisher_.shutdown();

  // Send SDOs.
  sendSdoReadServer_.shutdown();
  sendSdoWriteServer_.shutdown();

  // Drive info.
  getDriveInfoServer_.shutdown();
  setDriveInfoSerialNumberServer_.shutdown();
  setDriveInfoNameServer_.shutdown();
  setDriveInfoIdServer_.shutdown();
  setDriveInfoBootloaderVersionServer_.shutdown();

  // Flash storage.
  eraseFlashStorage_.shutdown();
  resetFlashStorageSections_.shutdown();

  // Calibration.
  getCalibrationStateServer_.shutdown();
  writeConfigurationServer_.shutdown();
  calibrateServer_.shutdown();
  resetCustomCalibrationsToFactoryServer_.shutdown();

  // Configuration.
  getMaxCurrentServer_.shutdown();
  setMaxCurrentServer_.shutdown();
  getMaxMotorVelocityServer_.shutdown();
  setMaxMotorVelocityServer_.shutdown();
  getJointPositionLimitsSdkServer_.shutdown();
  setJointPositionLimitsSdkServer_.shutdown();
  getJointPositionLimitsSoftServer_.shutdown();
  setJointPositionLimitsSoftServer_.shutdown();
  getJointPositionLimitsHardServer_.shutdown();
  setJointPositionLimitsHardServer_.shutdown();
  getControlGainsServer_.shutdown();
  setControlGainsServer_.shutdown();
  getErrorStateBehaviorServer_.shutdown();
  setErrorStateBehaviorServer_.shutdown();
  writeFactoryCalibrationServer_.shutdown();

  // Control.
  setGoalStateServer_.shutdown();
  sendControlwordServer_.shutdown();
  achieveJointPositionConfigurationServer_.shutdown();
}

void AnydriveManagerRos::commandsCb(const anydrive_msgs::CommandsConstPtr& commandsMsg) {
  std::vector<anydrive::Command> commands;
  for (const auto& commandMsg : commandsMsg->commands) {
    anydrive::Command command;
    readFromMessage(command, commandMsg);
    commands.push_back(command);
  }
  stageCommands(commands);
}

bool AnydriveManagerRos::publishWorkerCb(const any_worker::WorkerEvent& /*event*/) {
  uint64_t localCounter = 0;

  while (!shutdownPublishWorkerRequested_) {
    // Wait for the notification.
    {
      std::unique_lock<std::mutex> notifyPublishWorkerLock(notifyPublishWorkerMutex_);
      // Additional bool and counter check protecting against spurious wake ups.
      while (!shutdownPublishWorkerRequested_ && publishCounter_ <= localCounter) {
        notifyPublishWorkerCv_.wait(notifyPublishWorkerLock);
      }
      localCounter = publishCounter_;
    }

    // Break the loop without publishing if shutdown is requested.
    if (shutdownPublishWorkerRequested_) {
      break;
    }

    sendRos();
  }
  return true;
}

bool AnydriveManagerRos::eraseFlashStorageCb(anydrive_msgs::EraseFlashStorageRequest& req,
                                             anydrive_msgs::EraseFlashStorageResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Erase flash storage of all drives.
    eraseFlashStorage();
  } else {
    // Erase flash storage of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    anydrive->eraseFlashStorage();
  }
  return true;
}

bool AnydriveManagerRos::resetFlashStorageSectionsCb(anydrive_msgs::ResetFlashStorageSectionsRequest& req,
                                                     anydrive_msgs::ResetFlashStorageSectionsResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Reset flash storage sections of all drives.
    resetFlashStorageSections(req.flash_storage_sections);
  } else {
    // Reset flash storage sections of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    anydrive->resetFlashStorageSections(req.flash_storage_sections);
  }
  return true;
}

bool AnydriveManagerRos::getCalibrationStateCb(anydrive_msgs::GetCalibrationStateRequest& req,
                                               anydrive_msgs::GetCalibrationStateResponse& res) {
  // Get calibration state of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  anydrive::calibration::CalibrationState calibrationState;
  const bool success = anydrive->getCalibrationState(calibrationTypeMsgToEnum(req.calibration_type), calibrationState);
  res.calibration_state = calibrationState.all_;  // NOLINT(cppcoreguidelines-pro-type-union-access)
  return success;
}

bool AnydriveManagerRos::calibrateCb(anydrive_msgs::CalibrateRequest& req, anydrive_msgs::CalibrateResponse& /*res*/) {
  return calibrate(req.device_name, calibrationModeMsgToEnum(req.calibration_mode), req.gear_and_joint_encoder_homing_absolute != 0u,
                   req.gear_and_joint_encoder_homing_new_joint_position);
}

bool AnydriveManagerRos::resetCustomCalibrationsToFactoryCb(anydrive_msgs::ResetCustomCalibrationsToFactoryRequest& req,
                                                            anydrive_msgs::ResetCustomCalibrationsToFactoryResponse& /*res*/) {
  anydrive::calibration::CalibrationState calibrationState;
  calibrationState.all_ = req.calibration_state;  // NOLINT(cppcoreguidelines-pro-type-union-access)
  if (req.device_name.empty()) {
    // Reset custom calibrations to factory of all drives.
    return resetCustomCalibrationsToFactory(calibrationState);
  } else {
    // Reset custom calibrations to factory of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->resetCustomCalibrationsToFactory(calibrationState);
  }
}

bool AnydriveManagerRos::writeFactoryCalibrationCb(anydrive_msgs::WriteFactoryCalibrationRequest& req,
                                                   anydrive_msgs::WriteFactoryCalibrationResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Write factory calibration of all drives.
    return writeFactoryCalibration();
  } else {
    // Write factory calibration of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->writeFactoryCalibration();
  }
}

bool AnydriveManagerRos::getMaxCurrentCb(anydrive_msgs::GetMaxCurrentRequest& req, anydrive_msgs::GetMaxCurrentResponse& res) {
  // Get max current of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->getMaxCurrent(res.max_current);
}

bool AnydriveManagerRos::setMaxCurrentCb(anydrive_msgs::SetMaxCurrentRequest& req, anydrive_msgs::SetMaxCurrentResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set max current of all drives.
    return setMaxCurrent(req.max_current);
  } else {
    // Set max current of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setMaxCurrent(req.max_current);
  }
}

bool AnydriveManagerRos::getMaxMotorVelocityCb(anydrive_msgs::GetMaxMotorVelocityRequest& req,
                                               anydrive_msgs::GetMaxMotorVelocityResponse& res) {
  // Get max motor velocity of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->getMaxMotorVelocity(res.max_motor_velocity);
}

bool AnydriveManagerRos::setMaxMotorVelocityCb(anydrive_msgs::SetMaxMotorVelocityRequest& req,
                                               anydrive_msgs::SetMaxMotorVelocityResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set max motor velocity of all drives.
    return setMaxMotorVelocity(req.max_motor_velocity);
  } else {
    // Set max motor velocity of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setMaxMotorVelocity(req.max_motor_velocity);
  }
}

bool AnydriveManagerRos::getDirectionCb(anydrive_msgs::GetDirectionRequest& req, anydrive_msgs::GetDirectionResponse& res) {
  // Get direction of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->getDirection(res.direction);
}

bool AnydriveManagerRos::setDirectionCb(anydrive_msgs::SetDirectionRequest& req, anydrive_msgs::SetDirectionResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set direction of all drives.
    return setDirection(req.direction);
  } else {
    // Set direction of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setDirection(req.direction);
  }
}

bool AnydriveManagerRos::getJointPositionLimitsSdkCb(anydrive_msgs::GetJointPositionLimitsRequest& req,
                                                     anydrive_msgs::GetJointPositionLimitsResponse& res) {
  // Get SDK joint position limits of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  anydrive::common::Limits limits;
  const bool success = anydrive->getJointPositionLimitsSdk(limits);
  res.joint_position_limits.min = limits.min();
  res.joint_position_limits.max = limits.max();
  return success;
}

bool AnydriveManagerRos::setJointPositionLimitsSdkCb(anydrive_msgs::SetJointPositionLimitsRequest& req,
                                                     anydrive_msgs::SetJointPositionLimitsResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set SDK joint position limits of all drives.
    return setJointPositionLimitsSdk(anydrive::common::Limits(req.joint_position_limits.min, req.joint_position_limits.max));
  } else {
    // Set SDK joint position limits of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setJointPositionLimitsSdk(anydrive::common::Limits(req.joint_position_limits.min, req.joint_position_limits.max));
  }
}

bool AnydriveManagerRos::getJointPositionLimitsSoftCb(anydrive_msgs::GetJointPositionLimitsRequest& req,
                                                      anydrive_msgs::GetJointPositionLimitsResponse& res) {
  // Get soft joint position limits of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  anydrive::common::Limits limits;
  const bool success = anydrive->getJointPositionLimitsSoft(limits);
  res.joint_position_limits.min = limits.min();
  res.joint_position_limits.max = limits.max();
  return success;
}

bool AnydriveManagerRos::setJointPositionLimitsSoftCb(anydrive_msgs::SetJointPositionLimitsRequest& req,
                                                      anydrive_msgs::SetJointPositionLimitsResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set soft joint position limits of all drives.
    return setJointPositionLimitsSoft(anydrive::common::Limits(req.joint_position_limits.min, req.joint_position_limits.max));
  } else {
    // Set soft joint position limits of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setJointPositionLimitsSoft(anydrive::common::Limits(req.joint_position_limits.min, req.joint_position_limits.max));
  }
}

bool AnydriveManagerRos::getJointPositionLimitsHardCb(anydrive_msgs::GetJointPositionLimitsRequest& req,
                                                      anydrive_msgs::GetJointPositionLimitsResponse& res) {
  // Get hard joint position limits of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  anydrive::common::Limits limits;
  const bool success = anydrive->getJointPositionLimitsHard(limits);
  res.joint_position_limits.min = limits.min();
  res.joint_position_limits.max = limits.max();
  return success;
}

bool AnydriveManagerRos::setJointPositionLimitsHardCb(anydrive_msgs::SetJointPositionLimitsRequest& req,
                                                      anydrive_msgs::SetJointPositionLimitsResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set hard joint position limits of all drives.
    return setJointPositionLimitsHard(anydrive::common::Limits(req.joint_position_limits.min, req.joint_position_limits.max));
  } else {
    // Set hard joint position limits of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setJointPositionLimitsHard(anydrive::common::Limits(req.joint_position_limits.min, req.joint_position_limits.max));
  }
}

bool AnydriveManagerRos::getControlGainsCb(anydrive_msgs::GetControlGainsRequest& req, anydrive_msgs::GetControlGainsResponse& res) {
  // Get control gains of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  anydrive::mode::PidGainsF pidGains;
  const bool success = anydrive->getControlGains(modeMsgToEnum(req.mode), pidGains);
  res.pid_gains.p = pidGains.getP();
  res.pid_gains.i = pidGains.getI();
  res.pid_gains.d = pidGains.getD();
  return success;
}

bool AnydriveManagerRos::setControlGainsCb(anydrive_msgs::SetControlGainsRequest& req, anydrive_msgs::SetControlGainsResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set control gains of all drives.
    return setControlGains(modeMsgToEnum(req.mode), anydrive::mode::PidGainsF(req.pid_gains.p, req.pid_gains.i, req.pid_gains.d));
  } else {
    // Set control gains of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setControlGains(modeMsgToEnum(req.mode), anydrive::mode::PidGainsF(req.pid_gains.p, req.pid_gains.i, req.pid_gains.d));
  }
}

bool AnydriveManagerRos::getErrorStateBehaviorCb(anydrive_msgs::GetErrorStateBehaviorRequest& req,
                                                 anydrive_msgs::GetErrorStateBehaviorResponse& res) {
  // Get error state behavior of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->getErrorStateBehavior(res.error_state_behavior.error_state_behavior);
}

bool AnydriveManagerRos::setErrorStateBehaviorCb(anydrive_msgs::SetErrorStateBehaviorRequest& req,
                                                 anydrive_msgs::SetErrorStateBehaviorResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set error state behavior of all drives.
    return setErrorStateBehavior(req.error_state_behavior.error_state_behavior);
  } else {
    // Set error state behavior of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->setErrorStateBehavior(req.error_state_behavior.error_state_behavior);
  }
}

bool AnydriveManagerRos::writeConfigurationCb(anydrive_msgs::WriteConfigurationRequest& req,
                                              anydrive_msgs::WriteConfigurationResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Write configuration of all drives.
    return writeConfiguration();
  } else {
    // Write configuration of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    return anydrive->writeConfiguration();
  }
}

bool AnydriveManagerRos::setGoalStateCb(anydrive_msgs::SetFsmGoalStateRequest& req, anydrive_msgs::SetFsmGoalStateResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Set goal state enum of all drives.
    return setGoalStatesEnum(stateMsgToEnum(req.fsm_state));
  } else {
    // Set goal state enum of one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    anydrive->setGoalStateEnum(stateMsgToEnum(req.fsm_state));
  }
  return true;
}

bool AnydriveManagerRos::sendControlwordCb(anydrive_msgs::SendControlwordRequest& req, anydrive_msgs::SendControlwordResponse& /*res*/) {
  if (req.device_name.empty()) {
    // Send controlword to all drives.
    sendControlwords(req.controlword);
  } else {
    // Send controlword to one drive.
    anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
    if (!anydrive) {
      return false;
    }

    anydrive->sendControlword(req.controlword);
  }
  return true;
}

bool AnydriveManagerRos::achieveJointPositionConfigurationCb(anydrive_msgs::AchieveJointPositionConfigurationRequest& req,
                                                             anydrive_msgs::AchieveJointPositionConfigurationResponse& /*res*/) {
  return achieveJointPositionConfiguration(req.joint_position_configuration.name);
}

bool AnydriveManagerRos::sendSdoReadCb(anydrive_msgs::SendSdoReadRequest& /*req*/, anydrive_msgs::SendSdoReadResponse& /*res*/) {
  ANYDRIVE_WARN("AnydriveManagerRos::sendSdoReadCb not implemented");

  return false;
}

bool AnydriveManagerRos::sendSdoWriteCb(anydrive_msgs::SendSdoWriteRequest& /*req*/, anydrive_msgs::SendSdoWriteResponse& /*res*/) {
  ANYDRIVE_WARN("AnydriveManagerRos::sendSdoWriteCb not implemented");

  return false;
}

bool AnydriveManagerRos::getDriveInfoCb(anydrive_msgs::GetDriveInfoRequest& req, anydrive_msgs::GetDriveInfoResponse& res) {
  // Get calibration state of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  anydrive::DriveInfo driveInfo;
  const bool success = anydrive->getDriveInfo(driveInfo);
  res.drive_info.serial_number = driveInfo.getSerialNumber();
  res.drive_info.name = driveInfo.getName();
  res.drive_info.id = driveInfo.getId();
  res.drive_info.bootloader_version = driveInfo.getBootloaderVersion().toString();
  res.drive_info.firmware_version = driveInfo.getFirmwareVersion().toString();
  return success;
}

bool AnydriveManagerRos::setDriveInfoSerialNumber(anydrive_msgs::SetDriveInfoStringRequest& req,
                                                  anydrive_msgs::SetDriveInfoStringResponse& /*res*/) {
  // Get calibration state of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->setDriveInfoSerialNumber(req.data);
}

bool AnydriveManagerRos::setDriveInfoName(anydrive_msgs::SetDriveInfoStringRequest& req,
                                          anydrive_msgs::SetDriveInfoStringResponse& /*res*/) {
  // Get calibration state of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->setDriveInfoName(req.data);
}

bool AnydriveManagerRos::setDriveInfoId(anydrive_msgs::SetDriveInfoUint16Request& req, anydrive_msgs::SetDriveInfoUint16Response& /*res*/) {
  // Get calibration state of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->setDriveInfoId(req.data);
}

bool AnydriveManagerRos::setDriveInfoBootloaderVersion(anydrive_msgs::SetDriveInfoStringRequest& req,
                                                       anydrive_msgs::SetDriveInfoStringResponse& /*res*/) {
  // Get calibration state of one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return anydrive->setDriveInfoBootloaderVersion(anydrive::common::Version(req.data));
}

}  // namespace anydrive_ros
