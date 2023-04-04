#pragma once

#include "anydrive/Command.hpp"
#include "anydrive/Reading.hpp"
#include "anydrive/Statusword.hpp"
#include "anydrive/calibration/CalibrationState.hpp"
#include "anydrive/calibration/CalibrationTypeEnum.hpp"
#include "anydrive/calibration/parameter/Calibration.hpp"
#include "anydrive/common/FirmwareInfo.hpp"
#include "anydrive/common/Logger.hpp"
#include "anydrive/common/Version.hpp"
#include "anydrive/communication/CommunicationInterfaceBase.hpp"
#include "anydrive/configuration/Configuration.hpp"
#include "anydrive/fsm/StateMachine.hpp"
#include "anydrive/setup/Anydrive.hpp"
#include "anydrive/thread_sleep.hpp"
#include "anydrive/usings.hpp"

namespace anydrive {

//! Class implementing the interface to a single ANYdrive.
class Anydrive {
 protected:
  //! NaN abbreviation for convenience.
  static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
  //! Inf abbreviation for convenience.
  static constexpr double inf_ = std::numeric_limits<double>::infinity();

  //! Name of the device.
  std::string name_;
  //! Configuration of the device.
  configuration::Configuration configuration_;
  //! Abstract communication interface.
  communication::CommunicationInterfaceBasePtr communicationInterface_;
  //! Simple logger for calibrations.
  common::Logger<ReadingExtended> logger_;

  //! State machine of the device.
  fsm::StateMachine stateMachine_;

  //! Current statusword.
  Statusword statusword_;
  //! Remember if a statusword has been actively requested.
  std::atomic<bool> statuswordRequested_;

  //! Mutex protecting the reading.
  mutable std::recursive_mutex readingMutex_;
  //! Reading data.
  ReadingExtended reading_;

  //! Mutex protecting the command.
  mutable std::recursive_mutex stagedCommandMutex_;
  //! Command to be sent in the next update.
  Command stagedCommand_;
  //! Remember if a command has been staged.
  std::atomic<bool> commandIsStaged_;

  //! Prioritized callbacks called when a reading has been received.
  std::multimap<int, ReadingCb, std::greater<int>> readingCbs_;
  //! Prioritized callbacks called when an error appeared.
  std::multimap<int, ErrorCb, std::greater<int>> errorCbs_;
  //! Prioritized callbacks called when an error disappeared.
  std::multimap<int, ErrorRecoveredCb, std::greater<int>> errorRecoveredCbs_;
  //! Prioritized callbacks called when an fatal appeared.
  std::multimap<int, FatalCb, std::greater<int>> fatalCbs_;
  //! Prioritized callbacks called when an fatal disappeared.
  std::multimap<int, FatalRecoveredCb, std::greater<int>> fatalRecoveredCbs_;
  //! Prioritized callbacks called when the device disconnected.
  std::multimap<int, DeviceDisconnectedCb, std::greater<int>> deviceDisconnectedCbs_;
  //! Prioritized callbacks called when the device reconnected.
  std::multimap<int, DeviceReconnectedCb, std::greater<int>> deviceReconnectedCbs_;

  //! Remember if the joint position was in the SDK limit in the last time step.
  std::atomic<bool> wasWithinJointPositionLimitsSdk_;

 public:
  /*!
   * @name Setup and configuration
   *
   * Methods to set up and configure the device.
   */
  //@{

  /*!
   * Constructor.
   */
  Anydrive();

  /*!
   * Destructor.
   */
  virtual ~Anydrive() = default;

  /*!
   * Load the ANYdrive setup.
   * @param setup ANYdrive setup.
   * @return True if successful.
   */
  bool loadSetup(const setup::AnydrivePtr& setup);

  /*!
   * Get the name of the device.
   * @return Name of the device (Copy for thread safety).
   */
  std::string getName() const;

  /*!
   * Get the configuration of the device by reference.
   * @return Configuration of the device by reference.
   */
  configuration::Configuration& getConfiguration();

  /*!
   * Get the configuration of the device by const reference.
   * @return Configuration of the device by const reference.
   */
  const configuration::Configuration& getConfiguration() const;

  /*!
   * Set the communication interface.
   * @param communicationInterface Communication interface.
   */
  void setCommunicationInterface(const communication::CommunicationInterfaceBasePtr& communicationInterface);

  /*!
   * Get the communication interface.
   * @return Communication interface.
   */
  const communication::CommunicationInterfaceBasePtr& getCommunicationInterface() const;

  /*!
   * Add a callback called when a reading has been received.
   * @param readingCb Callback.
   * @param priority Priority.
   */
  void addReadingCb(const ReadingCb& readingCb, const int priority = 0);

  /*!
   * Add a callback called when an error appeared.
   * @param readingCb Callback.
   * @param priority Priority.
   */
  void addErrorCb(const ErrorCb& errorCb, const int priority = 0);

  /*!
   * Add a callback called when an error disappeared.
   * @param readingCb Callback.
   * @param priority Priority.
   */
  void addErrorRecoveredCb(const ErrorRecoveredCb& errorRecoveredCb, const int priority = 0);

  /*!
   * Add a callback called when a fatal appeared.
   * @param readingCb Callback.
   * @param priority Priority.
   */
  void addFatalCb(const FatalCb& fatalCb, const int priority = 0);

  /*!
   * Add a callback called when a fatal disappeared.
   * @param readingCb Callback.
   * @param priority Priority.
   */
  void addFatalRecoveredCb(const FatalRecoveredCb& fatalRecoveredCb, const int priority = 0);

  /*!
   * Add a callback called when the device disconnected.
   * @param readingCb Callback.
   * @param priority Priority.
   */
  void addDeviceDisconnectedCb(const DeviceDisconnectedCb& deviceDisconnectedCb, const int priority = 0);

  /*!
   * Add a callback called when a the device reconnected.
   * @param readingCb Callback.
   * @param priority Priority.
   */
  void addDeviceReconnectedCb(const DeviceReconnectedCb& deviceReconnectedCb, const int priority = 0);

  //@}

  /*!
   * @name Execution
   *
   * Methods to start up, update and shut down the device.
   */
  //@{

  /*!
   * Startup step 1: Startup/initialization procedures which do not require communication with the device.
   */
  virtual void startupWithoutCommunication();

  /*!
   * Startup step 2: Startup/initialization procedures which require communication with the device.
   */
  virtual void startupWithCommunication();

  /*!
   * Update step 1: Process the last received reading.
   * This includes updating the statusword and the state machine.
   */
  virtual void updateProcessReading();

  /*!
   * Update step 2: Send the staged command.
   * If a command is staged, it will be sent.
   */
  virtual void updateSendStagedCommand();

  /*!
   * Shutdown step 1: Shutdown/cleanup procedures which require communication with the device.
   */
  virtual void shutdownWithCommunication();

  /*!
   * Shutdown step 1: Shutdown/cleanup procedures which require communication with the device.
   * @param reachState        True: Block until the device is in the goal state, False: return after setting the goal state.
   * @param timeout           Maximal blocking time.
   * @param checkingFrequency Frequency at which is checked, whether the device has reached the goal state.
   */
  virtual void shutdownWithCommunication(const bool reachState, const double timeout, const double checkingFrequency);

  /*!
   * Shutdown step 2: Shutdown/cleanup procedures which do not require communication with the device.
   */
  virtual void shutdownWithoutCommunication();

  //@}

  /*!
   * @name State Machine
   *
   * Methods to interact with the device's state machine.
   */
  //@{

  /*!
   * Get the active state of the state machine.
   * @return Active state.
   */
  fsm::StateEnum getActiveStateEnum() const;

  /*!
   * Get the goal state of the state machine.
   * @return Goal state.
   */
  fsm::StateEnum getGoalStateEnum() const;

  /*!
   * Check if the device is in the goal state.
   * @return True if the device is in the goal state.
   */
  bool goalStateHasBeenReached() const;

  /*!
   * Set the goal state of the state machine.
   * @param goalStateEnum     Goal state.
   * @param reachState        True: Block until the device is in the goal state, False: return after setting the goal state.
   * @param timeout           Maximal blocking time.
   * @param checkingFrequency Frequency at which is checked, whether the device has reached the goal state.
   * @return True if non-blocking, or blocking and the device reached the goal state within the timeout.
   */
  bool setGoalStateEnum(const fsm::StateEnum goalStateEnum, const bool reachState = false, const double timeout = 5.0,
                        const double checkingFrequency = 100.0);

  /*!
   * Clear the current goal state of the state machine.
   */
  void clearGoalStateEnum();

  /*!
   * Call the prioritized error callbacks.
   */
  void errorCb();

  /*!
   * Call the prioritized error recovered callbacks.
   */
  void errorRecoveredCb();

  /*!
   * Check if the device is in the error state.
   * @return True if the device is in the error state.
   */
  bool deviceIsInErrorState();

  /*!
   * Call the prioritized fatal callbacks.
   */
  void fatalCb();

  /*!
   * Call the prioritized fatal recovered callbacks.
   */
  void fatalRecoveredCb();

  /*!
   * Check if the device is in the fatal state.
   * @return True if the device is in the fatal state.
   */
  bool deviceIsInFatalState();

  /*!
   * Call the device disconnected callbacks.
   */
  void deviceDisconnectedCb();

  /*!
   * Call the device reconnected callbacks.
   */
  void deviceReconnectedCb();

  /*!
   * Check if the device is missing.
   * @return True if the device is missing.
   */
  bool deviceIsMissing() const;

  /*!
   * Check whether the joint position is within the SDK limits.
   * @True if the device is within the SDK limits.
   */
  bool isWithinJointPositionLimitsSdk() const;

  /*!
   * Check whether the joint position is within the soft limits.
   * @True if the device is within the soft limits.
   */
  bool isWithinJointPositionLimitsSoft() const;

  /*!
   * Check whether the joint position is within the hard limits.
   * @True if the device is within the hard limits.
   */
  bool isWithinJointPositionLimitsHard() const;

  //@}

  /*!
   * @name Drive info
   *
   * Methods to access the info from the drive.
   */
  //@{

  /*!
   * Read the complete drive info from its flash storage.
   * Requires the device to be in the Configure state.
   * @param driveInfo Return argument, will contain the complete drive info.
   * @return True if successful.
   */
  bool getDriveInfo(DriveInfo& driveInfo) const;

  /*!
   * Read the device model from its flash storage.
   * @param model Return argument, will contain the device model.
   * @return True if successful.
   */
  bool getDriveModel(std::string& model) const;

  /*!
   * Read the device serial number from its flash storage.
   * Requires the device to be in the Configure state.
   * @param serialNumber Return argument, will contain the device serial number.
   * @return True if successful.
   */
  bool getDriveInfoSerialNumber(std::string& serialNumber) const;

  /*!
   * Write the device serial number to its flash storage.
   * Requires the device to be in the Configure state.
   * @param serialNumber The device serial number.
   * @return True if successful.
   */
  bool setDriveInfoSerialNumber(const std::string& serialNumber);

  /*!
   * Read the device name from its flash storage.
   * Requires the device to be in the Configure state.
   * @param name Return argument, will contain the device name.
   * @return True if successful.
   */
  bool getDriveInfoName(std::string& name) const;

  /*!
   * Write the device name to its flash storage.
   * Requires the device to be in the Configure state.
   * @param name The device name.
   * @return True if successful.
   */
  bool setDriveInfoName(const std::string& name);

  /*!
   * Read the device ID from its flash storage.
   * Requires the device to be in the Configure state.
   * @param id Return argument, will contain the device ID.
   * @return True if successful.
   */
  bool getDriveInfoId(uint16_t& id) const;

  /*!
   * Write the device ID to its flash storage.
   * Requires the device to be in the Configure state.
   * @param id The device ID.
   * @return True if successful.
   */
  bool setDriveInfoId(const uint16_t id);

  /*!
   * Read the device bootloader version from its flash storage.
   * Requires the device to be in the Configure state.
   * @param id Return argument, will contain the device bootloader version.
   * @return True if successful.
   */
  bool getDriveInfoBootloaderVersion(common::Version& bootloaderVersion) const;

  /*!
   * Write the device bootloader version to its flash storage.
   * Requires the device to be in the Configure state.
   * Note: This is not automated when flashing a new bootloader, so the version has to be set manually.
   * @param id The device bootloader version.
   * @return True if successful.
   */
  bool setDriveInfoBootloaderVersion(const common::Version& bootloaderVersion);

  /*!
   * Read the device firmware version from its flash storage.
   * Requires the device to be in the Configure state.
   * Note: This version is hard-coded in the firmware and cannot be changed.
   * @param id Return argument, will contain the device firmware version.
   * @return True if successful.
   */
  bool getDriveInfoFirmwareVersion(common::Version& firmwareVersion) const;

  /*!
   * Read the device firmware info.
   * @return True if successful.
   */
  bool getDriveFirmwareInfo(common::FirmwareInfo& firmwareInfo);

  /*!
   * Read the gearbox ratio of the device.
   * @param ratio The gearbox ratio, e.g., a 50:1 gearbox returns 50.
   * @return True if successful.
   */
  bool getGearboxRatio(uint32_t& ratio);

  //@}

  /*!
   * @name Flash storage
   *
   * Methods to control the flash storage on the device.
   */
  //@{

  /*!
   * Erase the flash storage including drive info, custom and factory calibrations and configuration.
   * Requires the device to be in the Configure state.
   * @return True if successful.
   */
  bool eraseFlashStorage();

  /*!
   * Reset a one or multiple flash storage sections.
   * Bit 1: Calibration.
   * Bit 2: Configuration.
   * Requires the device to be in the Configure state.
   * @param flashStorageSection Bit-field indicating the flash storage sections to reset.
   * @return True if successful.
   */
  bool resetFlashStorageSections(const uint8_t flashStorageSections);

  //@}

  /*!
   * @name Calibration
   *
   * Methods to interface with the device's calibration.
   *
   * The drive needs several calibrations in order to operate properly:
   *
   * - Motor encoder offset (for commutation)
   * - Motor encoder parameters (for commutation)
   * - Gear/joint encoder offset (for joint torque measurement)
   * - Gear and joint encoder homing (for gear and joint zero position)
   *
   * This set of calibrations parameters is stored twice on the flash storage,
   * one set for the custom and one set for the factory calibration. Calibrating the
   * device modifies the custom calibration. There are two methods to interact
   * with the factory calibration: resetCustomCalibrationsToFactory and
   * writeFactoryCalibration. Refer to their documentations for further details.
   */
  //@{

  /*!
   * Get the state of the custom calibration of the device.
   * The factory calibration is always complete.
   * @param calibrationTypeEnum Type of the calibration.
   * @param calibrationState    Return argument, will contain the calibration state.
   * @return True if successful.
   */
  bool getCalibrationState(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                           calibration::CalibrationState& calibrationState) const;

  /*!
   * Get all calibration values.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum Type of the calibration.
   * @param calibration         Return argument, will contain all calibration values.
   * @return True if successful.
   */
  bool getCalibration(const calibration::CalibrationTypeEnum calibrationTypeEnum, calibration::parameter::Calibration& calibration) const;

  /*!
   * Get the calibration values of the motor encoder offset calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum Type of the calibration.
   * @param motorEncoderOffset  Return argument, will contain the motor encoder offset calibration values.
   * @return True if successful.
   */
  bool getCalibrationMotorEncoderOffset(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        calibration::parameter::MotorEncoderOffset& motorEncoderOffset) const;

  /*!
   * Get the calibration values of the motor encoder parameters calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum    Type of the calibration.
   * @param motorEncoderParameters Return argument, will contain the motor encoder parameters calibration values.
   * @return True if successful.
   */
  bool getCalibrationMotorEncoderParameters(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                            calibration::parameter::MotorEncoderParameters& motorEncoderParameters) const;

  /*!
   * Get the calibration values of the gear/joint encoder offset calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum    Type of the calibration.
   * @param gearJointEncoderOffset Return argument, will contain the gear/joint encoder offset calibration values.
   * @return True if successful.
   */
  bool getCalibrationGearJointEncoderOffset(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                            calibration::parameter::GearJointEncoderOffset& gearJointEncoderOffset) const;

  /*!
   * Get the constant part of the calibration values of the gear/joint encoder offset calibration.
   * Requires the device to be in the Calibrate state.
   * @param constant Return argument, will contain the constant part of the gear/joint encoder offset calibration values.
   * @return True if successful.
   */
  bool getCalibrationGearJointEncoderOffsetConstant(int32_t& constant) const;

  /*!
   * Get the constant part of the calibration values of the gear/joint encoder offset calibration.
   * Requires the device to be in the Calibrate state.
   * @param constant constant part of the gear/joint encoder offset calibration values.
   * @return True if successful.
   */
  bool setCalibrationGearJointEncoderOffsetConstant(const int32_t constant);

  /*!
   * Get the calibration values of the gear and joint encoder homing calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum       Type of the calibration.
   * @param gearAndJointEncoderHoming Return argument, will contain the gear and joint encoder homing calibration values.
   * @return True if successful.
   */
  bool getCalibrationGearAndJointEncoderHoming(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                               calibration::parameter::GearAndJointEncoderHoming& gearAndJointEncoderHoming) const;

  /*!
   * Get the calibration values of the IMU gyroscope dc bias calibration.
   * @param calibrationTypeEnum Type of the calibration.
   * @param imuGyroscopeDcBias  Return argument, will contain the IMU gyroscope dc bias calibration values.
   * @return True if successful.
   */
  bool getCalibrationImuGyroscopeDcBias(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        calibration::parameter::ImuGyroscopeDcBias& imuGyroscopeDcBias) const;

  /*!
   * Set the calibration values of the spring stiffness calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum Type of the calibration.
   * @param springStiffness     Contains the spring stiffness calibration values.
   * @return True if successful.
   */
  bool setCalibrationSpringStiffness(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                     const calibration::parameter::SpringStiffness& springStiffness) const;

  /*!
   * Get the calibration values of the spring stiffness calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum Type of the calibration.
   * @param springStiffness     Return argument, will contain the spring stiffness calibration values.
   * @return True if successful.
   */
  bool getCalibrationSpringStiffness(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                     calibration::parameter::SpringStiffness& springStiffness) const;

  /*!
   * Set the calibration values of the friction estimation calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum  Type of the calibration.
   * @param frictionEstimation Contains the friction estimation calibration values.
   * @return True if successful.
   */
  bool setCalibrationFrictionEstimation(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        const calibration::parameter::FrictionEstimation& frictionEstimation) const;

  /*!
   * Get the calibration values of the friction estimation calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationTypeEnum  Type of the calibration.
   * @param frictionEstimation Return argument, will contain the friction estimation calibration values.
   * @return True if successful.
   */
  bool getCalibrationFrictionEstimation(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        calibration::parameter::FrictionEstimation& frictionEstimation) const;

  /*!
   * Reset the custom calibrations to the factory calibrations.
   * This will copy the factory calibration to the custom calibration.
   * Requires the device to be in the Calibrate state.
   * @param calibrationState Select which calibration(s) should be reset.
   * @return True if successful.
   */
  bool resetCustomCalibrationsToFactory(const calibration::CalibrationState calibrationState);

  /*!
   * Copy the custom calibration to the factory calibration.
   * Requires the device to be in the Calibrate state.
   * @return True if successful.
   */
  bool writeFactoryCalibration();

  /*!
   * Wait until a drive leaves calibration (blocking).
   * @param timeout           Maximal waiting time.
   * @param checkingFrequency Frequency to check the active mode.
   * @return True if drive left calibration state within the timeout.
   */
  bool waitForCalibrationDone(double timeout = 10.0, double checkingFrequency = 10.0) const;

  //@}

  /*!
   * @name Configuration
   *
   * Methods to interface with the device's configuration.
   */
  //@{

  /*!
   * Get the maximal current the motor may apply.
   * Requires the device to be in the Configure state.
   * @param maxCurrent Return argument, will contain the maximal current.
   * @return True if successful.
   */
  bool getMaxCurrent(double& maxCurrent) const;

  /*!
   * Set the maximal current the motor may apply.
   * Requires the device to be in the Configure state.
   * @param maxCurrent The maximal current.
   * @return True if successful.
   */
  bool setMaxCurrent(const double maxCurrent);

  /*!
   * Get the freeze current limit.
   * Requires the device to be in the Configure state.
   * @param current Return argument, will contain the freeze current limit.
   * @return True if successful.
   */
  bool getMaxFreezeCurrent(double& current);

  /*!
   * Set the freeze current limit.
   * Requires the device to be in the Configure state.
   * @param current Freeze current limit.
   * @return True if successful.
   */
  bool setMaxFreezeCurrent(const double current);

  /*!
   * Get the maximal velocity of the motor.
   * Requires the device to be in the Configure state.
   * @param maxMotorVelocity Return argument, will contain the maximal motor velocity.
   * @return True if successful.
   */
  bool getMaxMotorVelocity(double& maxMotorVelocity) const;

  /*!
   * Sets the maximal velocity of the motor.
   * Requires the device to be in the Configure state.
   * @param maxMotorVelocity The maximal motor velocity.
   * @return True if successful.
   */
  bool setMaxMotorVelocity(const double maxMotorVelocity);

  /*!
   * Get the maximal joint torque the actuator may apply.
   * Requires the device to be in the Configure state.
   * @param maxTorque Return argument, will contain the maximal torque.
   * @return True if successful.
   */
  bool getMaxJointTorque(double& maxJointTorque) const;

  /*!
   * Set the maximal joint torque the actuator may apply.
   * Requires the device to be in the Configure state.
   * @param maxTorque The maximal torque.
   * @return True if successful.
   */
  bool setMaxJointTorque(const double maxJointTorque);

  /*!
   * Get the integrator saturation of the current controller.
   * Requires the device to be in the Configure state.
   * @param saturation Return argument, will contain the integrator saturation of the current controller.
   * @return True if successful.
   */
  bool getCurrentIntegratorSaturation(double& saturation) const;

  /*!
   * Set the current integrator saturation the device is allowed to use.
   * Requires the device to be in the Configure state.
   * @param saturation The integrator saturation of the current controller.
   * @return True if successful.
   */
  bool setCurrentIntegratorSaturation(const double saturation);

  /*!
   * Get the integrator saturation of the joint torque controller.
   * Requires the device to be in the Configure state.
   * @param saturation Return argument, will contain the integrator saturation of the joint torque controller.
   * @return True if successful.
   */
  bool getJointTorqueIntegratorSaturation(double& saturation) const;

  /*!
   * Set the torque integrator saturation the device is allowed to use.
   * Requires the device to be in the Configure state.
   * @param saturation The integrator saturation of the joint torque controller.
   * @return True if successful.
   */
  bool setJointTorqueIntegratorSaturation(const double saturation);

  /*!
   * Get the rotation direction of the output shaft.
   * Requires the device to be in the Configure state.
   * @param direction Return argument, will contain the rotation direction.
   * @return True if successful.
   */
  bool getDirection(int16_t& direction) const;

  /*!
   * Set the rotation direction of the output shaft.
   * Requires the device to be in the Configure state.
   * @param direction The rotation direction.
   * @return True if successful.
   */
  bool setDirection(const int16_t direction);

  /*!
   * Get the joint position limits which are being checked in the SDK.
   * If these limits are exceeded, the error callbacks are called.
   * @param limits Return parameter, will contain the SDK joint positions limits.
   * @return True if successful.
   */
  bool getJointPositionLimitsSdk(common::Limits& limits) const;

  /*!
   * Set the joint position limits which are being checked in the SDK.
   * If these limits are exceeded, the error callbacks are called.
   * @param limits SDK joint positions limits.
   * @return True if successful.
   */
  bool setJointPositionLimitsSdk(const common::Limits& limits);

  /*!
   * Get the soft joint position limits which are being checked on the device.
   * If these limits are exceeded, the device enters the error state.
   * Requires the device to be in the Configure state.
   * @param limits Return parameter, will contain the soft joint positions limits.
   * @return True if successful.
   */
  bool getJointPositionLimitsSoft(common::Limits& limits) const;

  /*!
   * Set the soft joint position limits which are being checked on the device.
   * If these limits are exceeded, the device enters the error state.
   * Requires the device to be in the Configure state.
   * @param limits Soft joint positions limits.
   * @return True if successful.
   */
  bool setJointPositionLimitsSoft(const common::Limits& limits);

  /*!
   * Get the hard joint position limits which are being checked on the device.
   * If these limits are exceeded, the device enters the fatal state.
   * Requires the device to be in the Configure state.
   * @param limits Return parameter, will contain the hard joint positions limits.
   * @return True if successful.
   */
  bool getJointPositionLimitsHard(common::Limits& limits) const;

  /*!
   * Set the hard joint position limits which are being checked on the device.
   * If these limits are exceeded, the device enters the fatal state.
   * Requires the device to be in the Configure state.
   * @param limits Hard joint positions limits.
   * @return True if successful.
   */
  bool setJointPositionLimitsHard(const common::Limits& limits);

  /*!
   * Get the control gains for a certain mode.
   * Requires the device to be in the Configure state.
   * @param mode     Control mode to get the gains for.
   * @param pidGains Return argument, will contain the control gains.
   * @return True if successful.
   */
  bool getControlGains(const mode::ModeEnum mode, mode::PidGainsF& pidGains) const;

  /*!
   * Set the control gains for a certain mode.
   * Requires the device to be in the Configure state.
   * @param mode     Control mode to set the gains for.
   * @param pidGains Control gains.
   * @return True if successful.
   */
  bool setControlGains(const mode::ModeEnum mode, const mode::PidGainsF& pidGains);

  /*!
   * Get behavior of the device in the error state.
   * 0 = freeze the motor
   * 1 = disable the motor
   * Requires the device to be in the Configure state.
   * @param errorStateBehavior Return argument, will contain the behavior of the device.
   * @return True if successful.
   */
  bool getErrorStateBehavior(uint16_t& errorStateBehavior) const;

  /*!
   * Set behavior of the device in the error state.
   * 0 = freeze the motor
   * 1 = disable the motor
   * Requires the device to be in the Configure state.
   * @param errorStateBehavior Behavior of the device.
   * @return True if successful.
   */
  bool setErrorStateBehavior(const uint16_t errorStateBehavior);

  /*!
   * Get if IMU is disabled or enabled.
   * false = disable IMU
   * true = enable IMU
   * Requires the device to be in the Configure state.
   * @param enable IMU disabled or enabled.
   * @return True if successful.
   */
  bool getImuEnabled(bool& enabled) const;

  /*!
   * Disable/enable IMU.
   * false = disable IMU
   * true = enable IMU
   * Requires the device to be in the Configure state.
   * @param enable Disable/enable IMU.
   * @return True if successful.
   */
  bool setImuEnable(const bool enable);

  /*!
   * Get the range of the IMU accelerometer.
   * 0 = +/- 2g
   * 1 = +/- 4g
   * 2 = +/- 8g
   * 3 = +/- 16g
   * Requires the device to be in the Configure state.
   * @param range Accelerometer range of the IMU.
   * @return True if successful.
   */
  bool getImuAccelerometerRange(uint32_t& range);

  /*!
   * Set the range of the IMU accelerometer.
   * 0 = +/- 2g
   * 1 = +/- 4g
   * 2 = +/- 8g
   * 3 = +/- 16g
   * Requires the device to be in the Configure state.
   * @param range Accelerometer range of the IMU.
   * @return True if successful.
   */
  bool setImuAccelerometerRange(const uint32_t range);

  /*!
   * Get the range of the IMU gyroscope.
   * 0 = +/- 250dps
   * 1 = +/- 500dps
   * 2 = +/- 1000dps
   * 3 = +/- 2000dps
   * Requires the device to be in the Configure state.
   * @param range Gyroscope range of the IMU.
   * @return True if successful.
   */
  bool getImuGyroscopeRange(uint32_t& range);

  /*!
   * Set the range of the IMU gyroscope.
   * 0 = +/- 250dps
   * 1 = +/- 500dps
   * 2 = +/- 1000dps
   * 3 = +/- 2000dps
   * Requires the device to be in the Configure state.
   * @param range Gyroscope range of the IMU.
   * @return True if successful.
   */
  bool setImuGyroscopeRange(const uint32_t range);

  /*!
   * Get the fan mode.
   * 0 = OFF
   * 1 = Auto
   * 2 = Manual
   * @param mode Fan mode.
   * @return True if successful.
   */
  bool getFanMode(uint32_t& mode);

  /*!
   * Set the fan mode.
   * 0 = OFF
   * 1 = Auto
   * 2 = Manual
   * @param mode Fan mode.
   * @return True if successful.
   */
  bool setFanMode(const uint32_t mode);

  /*!
   * Get the fan intensity in manual and auto mode.
   * 0 = OFF
   * 1 = Minimal intensity
   * 10= Maximal intensity
   * @param intensity Fan intensity.
   * @return True if successful.
   */
  bool getFanIntensity(uint32_t& intensity);

  /*!
   * Get the fan lower temperature.
   * Requires the device to be in the Configure state.
   * @param temperature Fan lower temperature.
   * @return True if successful.
   */
  bool getFanLowerTemperature(float& temperature);

  /*!
   * Set the fan lower temperature.
   * Requires the device to be in the Configure state.
   * @param temperature Fan lower temperature.
   * @return True if successful.
   */
  bool setFanLowerTemperature(const float temperature);

  /*!
   * Get the fan upper temperature.
   * Requires the device to be in the Configure state.
   * @param temperature Fan upper temperature.
   * @return True if successful.
   */
  bool getFanUpperTemperature(float& temperature);

  /*!
   * Set the fan upper temperature.
   * Requires the device to be in the Configure state.
   * @param temperature Fan upper temperature.
   * @return True if successful.
   */
  bool setFanUpperTemperature(const float temperature);

  /*!
   * Enable/disable the brake output.
   * @param mode True enables, false disables
   * @return True if successful
   */
  bool setBrakeMode(const bool mode);

  /*!
   * Get the state of the brake output
   * @param mode True if brake enabled, false if disabled
   * @return True if successful
   */
  bool getBrakeMode(bool& mode);

  /*!
   * Get the duty cycle of the brake; mapped to [0, 1].
   * @param d Brake output duty cycle
   * @return True if successful
   */
  bool getBrakeDuty(float& d);

  /*!
   * Get the gear/joint velocity filter type.
   * Requires the device to be in the Configure state.
   * @param type Filter type.
   * @return True if successful.
   */
  bool getGearJointVelocityFilterType(uint32_t& type);

  /*!
   * Set the gear/joint velocity filter type.
   * Requires the device to be in the Configure state.
   * @param type Filter type.
   * @return True if successful.
   */
  bool setGearJointVelocityFilterType(const uint32_t type);

  /*!
   * Get the gear/joint velocity filter KF noise varaiance.
   * Requires the device to be in the Configure state.
   * @param variance Gear/joint velocity filter KF noise varaiance.
   * @return True if successful.
   */
  bool getGearJointVelocityKfNoiseVariance(float& variance);

  /*!
   * Set the gear/joint velocity filter KF noise varaiance.
   * Requires the device to be in the Configure state.
   * @param variance Gear/joint velocity filter KF noise varaiance.
   * @return True if successful.
   */
  bool setGearJointVelocityKfNoiseVariance(const float variance);

  /*!
   * Get the gear/joint velocity filter KF lambda^2.
   * Requires the device to be in the Configure state.
   * @param lambda Gear/joint velocity filter KF lambda^2.
   * @return True if successful.
   */
  bool getGearJointVelocityKfLambda2(float& lambda);

  /*!
   * Set the gear/joint velocity filter KF lambda^2.
   * Requires the device to be in the Configure state.
   * @param lambda Gear/joint velocity filter KF lambda^2.
   * @return True if successful.
   */
  bool setGearJointVelocityKfLambda2(const float lambda);

  /*!
   * Get the gear/joint velocity filter KF gamma.
   * Requires the device to be in the Configure state.
   * @param gamma Gear/joint velocity filter KF gamma.
   * @return True if successful.
   */
  bool getGearJointVelocityKfGamma(float& gamma);

  /*!
   * Set the gear/joint velocity filter KF gamma.
   * Requires the device to be in the Configure state.
   * @param gamma Gear/joint velocity filter KF gamma.
   * @return True if successful.
   */
  bool setGearJointVelocityKfGamma(const float gamma);

  /*!
   * Get the gear/joint velocity EMA alpha.
   * Requires the device to be in the Configure state.
   * @param alpha Gear/joint velocity EMA alpha.
   * @return True if successful.
   */
  bool getGearJointVelocityEmaAlpha(float& alpha);

  /*!
   * Set the gear/joint velocity EMA alpha.
   * Requires the device to be in the Configure state.
   * @param alpha Gear/joint velocity EMA alpha.
   * @return True if successful.
   */
  bool setGearJointVelocityEmaAlpha(const float alpha);

  /*!
   * Get the joint velocity for acceleration filter type.
   * Requires the device to be in the Configure state.
   * @param type Filter type.
   * @return True if successful.
   */
  bool getJointVelocityForAccelerationFilterType(uint32_t& type);

  /*!
   * Set the joint velocity for acceleration filter type.
   * Requires the device to be in the Configure state.
   * @param type Filter type.
   * @return True if successful.
   */
  bool setJointVelocityForAccelerationFilterType(const uint32_t type);

  /*!
   * Get the joint velocity for acceleration KF noise varaiance.
   * Requires the device to be in the Configure state.
   * @param variance Joint velocity for acceleration KF noise varaiance.
   * @return True if successful.
   */
  bool getJointVelocityForAccelerationKfNoiseVariance(float& variance);

  /*!
   * Set the joint velocity for acceleration KF noise varaiance.
   * Requires the device to be in the Configure state.
   * @param variance Joint velocity for acceleration KF noise varaiance.
   * @return True if successful.
   */
  bool setJointVelocityForAccelerationKfNoiseVariance(const float variance);

  /*!
   * Get the joint velocity for acceleration KF lambda^2.
   * Requires the device to be in the Configure state.
   * @param lambda Joint velocity for acceleration KF lambda^2.
   * @return True if successful.
   */
  bool getJointVelocityForAccelerationKfLambda2(float& lambda);

  /*!
   * Set the joint velocity for acceleration KF lambda^2.
   * Requires the device to be in the Configure state.
   * @param lambda Joint velocity for acceleration KF lambda^2.
   * @return True if successful.
   */
  bool setJointVelocityForAccelerationKfLambda2(const float lambda);

  /*!
   * Get the joint velocity for acceleration KF gamma.
   * Requires the device to be in the Configure state.
   * @param gamma Joint velocity for acceleration KF gamma.
   * @return True if successful.
   */
  bool getJointVelocityForAccelerationKfGamma(float& gamma);

  /*!
   * Set the joint velocity for acceleration KF gamma.
   * Requires the device to be in the Configure state.
   * @param gamma Joint velocity for acceleration KF gamma.
   * @return True if successful.
   */
  bool setJointVelocityForAccelerationKfGamma(const float gamma);

  /*!
   * Get the joint velocity for acceleration EMA alpha.
   * Requires the device to be in the Configure state.
   * @param alpha Joint velocity for acceleration EMA alpha.
   * @return True if successful.
   */
  bool getJointVelocityForAccelerationEmaAlpha(float& alpha);

  /*!
   * Set the joint velocity for acceleration EMA alpha.
   * Requires the device to be in the Configure state.
   * @param alpha Joint velocity for acceleration EMA alpha.
   * @return True if successful.
   */
  bool setJointVelocityForAccelerationEmaAlpha(const float alpha);

  /*!
   * Get the joint acceleration filter type.
   * Requires the device to be in the Configure state.
   * @param type Filter type.
   * @return True if successful.
   */
  bool getJointAccelerationFilterType(uint32_t& type);

  /*!
   * Set the joint acceleration filter type.
   * Requires the device to be in the Configure state.
   * @param type Filter type.
   * @return True if successful.
   */
  bool setJointAccelerationFilterType(const uint32_t type);

  /*!
   * Get the joint acceleration KF noise varaiance.
   * Requires the device to be in the Configure state.
   * @param variance Joint acceleration KF noise varaiance.
   * @return True if successful.
   */
  bool getJointAccelerationKfNoiseVariance(float& variance);

  /*!
   * Set the joint acceleration KF noise varaiance.
   * Requires the device to be in the Configure state.
   * @param variance Joint acceleration KF noise varaiance.
   * @return True if successful.
   */
  bool setJointAccelerationKfNoiseVariance(const float variance);

  /*!
   * Get the joint acceleration KF lambda^2.
   * Requires the device to be in the Configure state.
   * @param lambda Joint acceleration KF lambda^2.
   * @return True if successful.
   */
  bool getJointAccelerationKfLambda2(float& lambda);

  /*!
   * Set the joint acceleration KF lambda^2.
   * Requires the device to be in the Configure state.
   * @param lambda Joint acceleration KF lambda^2.
   * @return True if successful.
   */
  bool setJointAccelerationKfLambda2(const float lambda);

  /*!
   * Get the joint acceleration KF gamma.
   * Requires the device to be in the Configure state.
   * @param gamma Joint acceleration KF gamma.
   * @return True if successful.
   */
  bool getJointAccelerationKfGamma(float& gamma);

  /*!
   * Set the joint acceleration KF gamma.
   * Requires the device to be in the Configure state.
   * @param gamma Joint acceleration KF gamma.
   * @return True if successful.
   */
  bool setJointAccelerationKfGamma(const float gamma);

  /*!
   * Get the joint acceleration EMA alpha.
   * Requires the device to be in the Configure state.
   * @param alpha Joint acceleration EMA alpha.
   * @return True if successful.
   */
  bool getJointAccelerationEmaAlpha(float& alpha);

  /*!
   * Set the joint acceleration EMA alpha.
   * Requires the device to be in the Configure state.
   * @param alpha Joint acceleration EMA alpha.
   * @return True if successful.
   */
  bool setJointAccelerationEmaAlpha(const float alpha);

  /*!
   * Write the configuration to the flash storage to become permanent.
   * @return True if successful.
   */
  bool writeConfiguration();

  //@}

  /*!
   * @name Status
   *
   * Methods to access the state of the device.
   */
  //@{

  /*!
   * Get the statusword of the device.
   * @return Statusword of the device.
   */
  Statusword getStatusword() const;

  //@}

  /*!
   * @name Control
   *
   * Methods to run controllers on the device.
   *
   * Commands are sent with a constant frequency, at which the update functions are
   * called. A new command is not sent using a "send" method directly (which would
   * interrupt the constant communication frequency), but with a "stage" method.
   * Staging a command means to save it for sending in the next update step. Only
   * one command can be staged at the time. Two stages after each other without an
   * update step in between will cause in the first stage to be overwritten without
   * being sent.
   */
  //@{

  /*!
   * Send a controlword to the device.
   * @param controlwordId ID of the controlword to send.
   */
  void sendControlword(const uint16_t controlwordId);

  /*!
   * Stage a disable command.
   * Convenience function.
   */
  void stageDisable();

  /*!
   * Stage a freeze command.
   * Convenience function.
   */
  void stageFreeze();

  /*!
   * Stage a zero joint torque command.
   * Convenience function.
   */
  void stageZeroJointTorque();

  /*!
   * Stage a command.
   * @param command Command to stage.
   */
  void stageCommand(const Command& command);

  /*!
   * Check if a command is staged.
   * @return True if a command is staged.
   */
  bool commandIsStaged() const;

  /*!
   * Wait until a certain mode is active (blocking).
   * @param mode              Mode to wait for.
   * @param timeout           Maximal waiting time.
   * @param checkingFrequency Frequency to check the active mode.
   * @return True if the mode is active within the timeout.
   */
  bool waitForMode(const mode::ModeEnum& mode, double timeout = 1.0, double checkingFrequency = 100.0) const;

  /*!
   * Set the fan intensity in manual mode (ignored in auto mode).
   * 0 = OFF
   * 1 = Minimal intensity
   * 10= Maximal intensity
   * @param intensity Fan intensity.
   * @return True if successful.
   */
  bool setFanIntensity(const uint32_t intensity);

  /*!
   * Set the duty cycle of the brake output signal.
   * Note that when 0% duty cycle is set, there are still (very short)
   * pulses appearing at the brake output. Disable the brake for complete
   * shutdown of the brake output.
   * @param d Duty cycle, range: 0%-100%
   * @return True if successful
   */
  bool setBrakeDuty(const float d);

  //@}

  /*!
   * @name Readings
   *
   * Methods to access the readings of the device.
   */
  //@{

  /*!
   * Get the newest reading which has been received.
   * @return Newest reading.
   */
  ReadingExtended getReading() const;

  /*!
   * Get the newest reading which has been received.
   * @param reading Return argument, will contain the newest reading.
   */
  void getReading(ReadingExtended& reading) const;

  //@}

  /*!
   * @name Logging
   *
   * Methods to control logging.
   *
   * A simple logging mechanism allows to collect readings over time.
   * Useful for calibration or testing.
   */
  //@{

  /*!
   * Start a new log. Skips if a log with the given name already exists.
   * @param name Name of the new log.
   */
  void startLog(const std::string& name);

  /*!
   * Stop the active log.
   */
  void stopLog();

  /*!
   * Get a log by name.
   * @param name Name of the log.
   * @return Log.
   */
  Log<ReadingExtended> getLog(const std::string& name) const;

  /*!
   * Clear all logs.
   */
  void clearLogs();

  //@}

  /*!
   * @name Real Time Data Logging (RTDL)
   *
   * Methods to control RTDL.
   *
   * A simple logging mechanism allows to log data in real time on the drive and
   * stream it afterwards. It is possible to log data either with 2.5 kHz or
   * 10 kHz. The streaming frequency should be lower than the communication
   * interface frequency.
   */
  //@{

  /*!
   * Get if RTDL is enabled or disabled.
   * false = disable RTDL
   * true = enable RTDL
   * @param enabled RTDL enabled or disabled.
   * @return True if successful.
   */
  bool getRtdlEnabled(bool& enabled);

  /*!
   * Enable or disable RTDL on the drive.
   * false = disable RTDL
   * true = enable RTDL
   * @param enable Enable or disable RTDL.
   * @return True if successful.
   */
  bool setRtdlEnable(const bool enable);

  /*!
   * Set the RTDL command.
   * 0 = stop
   * 1 = reset
   * 2 = log
   * 3 = stream
   * @param command RTDL command.
   * @return True if successful.
   */
  bool setRtdlCommand(const uint16_t command);

  /*!
   * Get the RTDL status.
   * 0 = idle
   * 1 = logging
   * 2 = streaming
   * @param status Current RTDL status.
   * @return True if successful.
   */
  bool getRtdlStatus(uint16_t& status);

  /*!
   * Get the RTDL logging frequency.
   * 0 = 2.5 kHz
   * 1 = 10 kHz
   * @param frequency RTDL logging frequency.
   * @return True if successful.
   */
  bool getRtdlLoggingFrequency(uint16_t& frequency);

  /*!
   * Set the RTDL logging frequency.
   * 0 = 2.5 kHz
   * 1 = 10 kHz
   * @param frequency RTDL logging frequency.
   * @return True if successful.
   */
  bool setRtdlLoggingFrequency(const uint16_t frequency);

  /*!
   * Get the RTDL streaming frequency in Hz.
   * @param frequency RTDL streaming frequency in Hz.
   * @return True if successful.
   */
  bool getRtdlStreamingFrequency(uint16_t& frequency);

  /*!
   * Set the RTDL streaming frequency in Hz. This frequency should be lower than
   * the communication frequency, otherwise data might be lost.
   * @param frequency RTDL streaming frequency in Hz.
   * @return True if successful.
   */
  bool setRtdlStreamingFrequency(const uint16_t frequency);

  /*!
   * Get the last logged timestamp, this can be used to check if the streaming
   * is finished.
   * @param timestamp Last timestamp of the data logging in 25e-6 seconds.
   * @return True if successful.
   */
  bool getRtdlLastTimestamp(uint64_t& timestamp);

  /*!
   * @param calibrationTypeId Id of the calibration type
   * @return True if successful.
   */
  bool getCalibrationTypeId(uint16_t& calibrationTypeId);

  /*!
   * @param calibrationTypeId Id of the calibration type
   * @return True if successful.
   */
  bool setCalibrationTypeId(const uint16_t calibrationTypeId);

  bool getLockStatus(uint32_t& status);
  bool sendPassword(const std::string& password);

  /*!
   *  Clear the logged data from the drive flash storage.
   *  Requires the device to be in the Configure state
   *  Also, the drive must be unlocked
   * @return True if successful
   */
  bool clearLoggedData();

  /*!
   * Transfer the logged data from the internal storage to
   * the EtherCAT dictionary such that it can subsequently be read out.
   * Requires the device to be in the Configure state
   * Also, the drive must be unlocked
   * @return True if successful
   */
  bool refreshLoggedData();

  /*!
   * Reads the number of histogram bins used by the firmware in the datalogger.
   * @param numHistogramBins Number of bins per histogram
   * @return True if successful
   */
  bool getNumHistogramBins(int& numHistogramBins);

  /*!
   * Read the logged histograms from the drive.
   * Unit: Nm, rad/s, W and °C.
   * Logged data must be refreshed before to get most recent data.
   * @param histTorque, histVelocity, histCurrent, histTemperature Vectors for the histograms Can be of any length.
   * @return True if successful
   */
  bool readHistograms(std::vector<uint64_t>& histTorque, std::vector<uint64_t>& histVelocity, std::vector<uint64_t>& histCurrent,
                      std::vector<uint64_t>& histTemperature);

  /*!
   * Obtain the active and total logged operating times from the drive.
   * Unit: micro-second
   * @param timeTotal
   * @param timeActive
   * @return True if successful
   */
  bool readOperatingTimes(uint64_t& timeTotal, uint64_t& timeActive);

  /*!
   * Obtain the total logged joint travel distance.
   * Unit: rad
   * @param jointTravel
   * @return True if successful
   */
  bool readJointTravel(double& jointTravel);

  /*!
   * Obtain the outermost edges of the logged data histograms. This is firmware-specific and needed to evaluate the histogram data.
   * The parameters should be self-explanatory. All values above and below the thresholds are sorted into the corresponding
   * outermost histogram bins.
   * @param torque_upper
   * @param torque_lower
   * @param velocity_upper
   * @param velocity_lower
   * @param current_upper
   * @param current_lower
   * @param temperature_upper
   * @param temperature_lower
   * @return True if successful
   */
  bool readHistogramEdges(float& torque_upper, float& torque_lower, float& velocity_upper, float& velocity_lower, float& current_upper,
                          float& current_lower, float& temperature_upper, float& temperature_lower);

  //@}

  /*!
   * @name Debugging
   *
   * Methods for debugging.
   */
  //@{

  /*!
   * Print device informations.
   * @param prefix Prefix to print.
   */
  void printInfo(const std::string& prefix = "") const;

  //@}

 protected:
  /*!
   * Request and set a new statusword if it is empty or outdated.
   */
  void requestAndSetStatusword();

  /*!
   * Set a new statusword.
   * Prints warnings, errors and fatals if the statusword changed.
   */
  void setStatusword(Statusword& statusword);
};

using AnydrivePtr = std::shared_ptr<Anydrive>;

}  // namespace anydrive
