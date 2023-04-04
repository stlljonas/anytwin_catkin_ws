#pragma once

#include <unistd.h>
#include <mutex>
#include <unordered_map>

#include <yaml_tools/YamlNode.hpp>

#include <any_worker/Worker.hpp>

#include "anydrive/Anydrive.hpp"
#include "anydrive/JointPositionConfigurationManager.hpp"
#include "anydrive/calibration/routine/CalibrationBase.hpp"
#include "anydrive/communication/CommunicationManagerBase.hpp"
#include "anydrive/setup/Setup.hpp"

namespace anydrive {

//! Class implementing a manager for multiple ANYdrives.
class AnydriveManager {
 public:
  //! ANYdrives container.
  using Anydrives = std::vector<AnydrivePtr>;

 protected:
  /*!
   * Flag for running in standalone mode.
   * True: A worker is instantiated, which calls update() in every time step.
   * False: update() has to be periodically called from the outside.
   */
  bool standalone_ = true;
  /*!
   * Flag indicating whether a shutdown has been requested.
   * True: handleSignal() is bound to signal handler.
   * False: default system signal handling is active.
   */
  bool installSignalHandler_ = true;

  //! Update worker.
  std::shared_ptr<any_worker::Worker> updateWorker_;

  //! Time step at which the update() function is called.
  double timeStep_ = 0.0;

  //! Mutex instead of atomic_bool to block during starting up and shutting down procedures.
  mutable std::recursive_mutex isRunningMutex_;
  //! Flag indicating whether the manager is running. True between startup() and shutdown() calls.
  bool isRunning_ = false;
  //! Flag indicating whether a shutdown has been requested.
  std::atomic<bool> shutdownRequested_;

  //! Abstract communication manager.
  communication::CommunicationManagerBasePtr communicationManager_;
  //! List of ANYdrives.
  Anydrives anydrives_;

  //! Calibration which is currently running.
  calibration::routine::CalibrationBasePtr runningCalibration_;
  //! Manager for commanding multiple ANYdrives to a given joint position configuration.
  JointPositionConfigurationManager jointPositionConfigurationManager_;

 public:
  /*!
   * @name Setup and configuration
   *
   * Methods to set up and configure the manager.
   */
  //@{

  /*!
   * Constructor.
   * @param standalone            Flag for running in standalone mode.
   * @param installSignalHandler  Flag indicating whether a signal handler shall be installed.
   * @param timeStep              Time step at which the update() function is called.
   */
  AnydriveManager(const bool standalone, const bool installSignalHandler, const double timeStep);

  /*!
   * Destructor.
   */
  virtual ~AnydriveManager();

  /*!
   * Static function to set the priority of the process which calls it.
   * @param priority Priority.
   * @return True if successful.
   */
  static void setProcessPriority(const int priority);

  /*!
   * Change the time step given in the constructor.
   * @param timeStep Time step.
   */
  void setTimeStep(const double timeStep);

  /*!
   * Get the time step.
   * @return Time step.
   */
  double getTimeStep() const;

  /*!
   * Set the communication manager.
   * @param communicationManager Communication manager.
   */
  void setCommunicationManager(const communication::CommunicationManagerBasePtr& communicationManager);

  /*!
   * Get the communication manager.
   * @return Communication manager.
   */
  const communication::CommunicationManagerBasePtr& getCommunicationManager() const;

  /*!
   * Get the joint position configuration manager by reference.
   * @return Joint position configuration manager by reference.
   */
  JointPositionConfigurationManager& getJointPositionConfigurationManager();

  /*!
   * Get the joint position configuration manager by const reference.
   * @return Joint position configuration manager by const reference.
   */
  const JointPositionConfigurationManager& getJointPositionConfigurationManager() const;

  /*!
   * Load the setup from a file.
   * If the setup file is empty, the default setup is loaded.
   * @param setupFile Absolute path to the setup file.
   * @return True if successful.
   */
  bool loadSetup(const std::string& setupFile = "");

  /*!
   * Create a new default setup.
   * @return New default setup.
   */
  virtual setup::SetupPtr createSetup() const;

  /*!
   * Load the setup from a struct.
   * @param setup Setup data.
   * @return True if successful.
   */
  bool loadSetup(const setup::SetupPtr& setup);

  /*!
   * Add an ANYdrive to the manager.
   * @param anydrive ANYdrive to add.
   * @return True if successful.
   */
  virtual bool addAnydrive(const AnydrivePtr& anydrive);

  /*!
   * Check if an ANYdrive with a given name exists.
   * @param name Name of the ANYdrive.
   * @return True if existing.
   */
  bool anydriveExists(const std::string& name) const;

  /*!
   * Get an ANYdrive by name. If unsure, first check if the ANYdrive exists with anydriveExists(..).
   * @param name Name of the ANYdrive.
   * @return Pointer to the ANYdrive.
   */
  AnydrivePtr getAnydrive(const std::string& name) const;

  /*!
   * Get all ANYdrives.
   * @return List of all ANYdrives.
   */
  Anydrives getAnydrives() const;

  /*!
   * Get the number of ANYdrives.
   * @return Number of ANYdrives.
   */
  unsigned int getNumberOfAnydrives() const;

  /*!
   * Add a callback when a new reading is available.
   * @param readingCb Callback function.
   * @param priority  Priority determining the order how the callbacks are called.
   */
  void addReadingCb(const ReadingCb& readingCb, const int priority = 0);

  /*!
   * Add a callback when the device enters the Error state.
   * @param readingCb Callback function.
   * @param priority  Priority determining the order how the callbacks are called.
   */
  void addErrorCb(const ErrorCb& errorCb, const int priority = 0);

  /*!
   * Add a callback when the device leaves the Error state.
   * @param readingCb Callback function.
   * @param priority  Priority determining the order how the callbacks are called.
   */
  void addErrorRecoveredCb(const ErrorRecoveredCb& errorRecoveredCb, const int priority = 0);

  /*!
   * Add a callback when the device enters the Fatal state.
   * @param readingCb Callback function.
   * @param priority  Priority determining the order how the callbacks are called.
   */
  void addFatalCb(const FatalCb& fatalCb, const int priority = 0);

  /*!
   * Add a callback when the device leaves the Fatal state.
   * @param readingCb Callback function.
   * @param priority  Priority determining the order how the callbacks are called.
   */
  void addFatalRecoveredCb(const FatalRecoveredCb& fatalRecoveredCb, const int priority = 0);

  /*!
   * Add a callback when the device enters the DeviceDisconnected state.
   * @param readingCb Callback function.
   * @param priority  Priority determining the order how the callbacks are called.
   */
  void addDeviceDisconnectedCb(const DeviceDisconnectedCb& deviceDisconnectedCb, const int priority = 0);

  /*!
   * Add a callback when the device leaves the DeviceDisconnected state.
   * @param readingCb Callback function.
   * @param priority  Priority determining the order how the callbacks are called.
   */
  void addDeviceReconnectedCb(const DeviceReconnectedCb& deviceReconnectedCb, const int priority = 0);

  //@}

  /*!
   * @name Execution
   *
   * Methods to start up, update and shut down the manager.
   */
  //@{

  /*!
   * Start the ANYdrive Manager
   * @return True if successful.
   */
  virtual bool startup();

  /*!
   * Start the ANYdrive Manager
   * @param startupRetries Number of startup retries, zero for infinity.
   * @return True if successful.
   */
  virtual bool startup(unsigned int startupRetries);

  /*!
   * Update the ANYdrive Manager, calling all update steps sequentially.
   * If standalone == true, this method is called automatically.
   * @return True if successful.
   */
  virtual bool update();

  /*!
   * Update step 1: Read new messages if available.
   * Can be used instead of update() for more fine-grained control over the update procedure.
   */
  virtual void updateCommunicationManagerReadMessages();

  /*!
   * Update step 2: Process the received readings.
   * This includes updating the statusword and the state machine.
   * Can be used instead of update() for more fine-grained control over the update procedure.
   */
  virtual void updateProcessReadings();

  /*!
   * Update step 3: Stage commands if the joint position configuration manager is active.
   * Can be used instead of update() for more fine-grained control over the update procedure.
   */
  virtual void updateStageCommands();

  /*!
   * Update step 4: Send the staged commands.
   * If commands are staged, they will be sent.
   * Can be used instead of update() for more fine-grained control over the update procedure.
   */
  virtual void updateSendStagedCommands();

  /*!
   * Update step 5: Write new messages if available.
   * Can be used instead of update() for more fine-grained control over the update procedure.
   */
  virtual void updateCommunicationManagerWriteMessages();

  /*!
   * Shutdown the ANYdrive Manager.
   */
  virtual void shutdown();

  /*!
   * Check if the ANYdrive Manager is running.
   * @return True if running.
   */
  bool isRunning() const;

  /*!
   * Check if a shutdown has been requested.
   * @return True if the shutdown has been requested.
   */
  bool shutdownRequested() const;

  /*!
   * Check if communication is ok.
   * @return True if communication is ok.
   */
  bool isCommunicationOk() const;

  /*!
   * Return working counter too low count.
   * @return Working counter too low count.
   */
  unsigned int getWorkingCounterTooLowCount() const;

  //@}

  /*!
   * @name State Machine
   *
   * Methods to interact with the device state machines.
   */
  //@{

  /*!
   * Set the goal state of the state machines of all devices.
   * @param goalStateEnum     Goal state.
   * @param reachState        True: Block until the devices are in the goal state, False: return after setting the goal state.
   * @param timeout           Maximal blocking time.
   * @param checkingFrequency Frequency at which is checked, whether the devices have reached the goal state.
   * @return True if non-blocking, or blocking and the devices reached the goal state within the timeout.
   */
  bool setGoalStatesEnum(const fsm::StateEnum goalStateEnum, const bool reachStates = false, const double timeout = 5.0,
                         const double checkingFrequency = 100.0);

  /*!
   * Clear the current goal state of the state machines of all devices.
   */
  void clearGoalStatesEnum();

  /*!
   * Check if all devices are connected.
   * @return True if all devices are connected.
   */
  bool allDevicesAreConnected() const;

  /*!
   * Check if all devices are in a given state.
   * @param stateEnum State to check.
   * @return True if all devices are in the given state.
   */
  bool allDevicesAreInTheState(const fsm::StateEnum stateEnum) const;

  /*!
   * Check if all devices if the are in the same state and get it.
   * @param stateEnum Return argument, will contain the common state. NA if not all devices are in the same state.
   * @return True if all devices are in the same state.
   */
  bool allDevicesAreInTheSameState(fsm::StateEnum& stateEnum) const;

  /*!
   * Check if all devices are in a given mode.
   * @param modeEnum Mode to check.
   * @return True if all devices are in the given mode.
   */
  bool allDevicesAreInTheMode(const mode::ModeEnum modeEnum) const;

  /*!
   * Check if all devices if the are in the same mode and get it.
   * @param modeEnum Return argument, will contain the common mode. NA if not all devices are in the same mode.
   * @return True if all devices are in the same mode.
   */
  bool allDevicesAreInTheSameMode(mode::ModeEnum& modeEnum) const;

  /*!
   * Check if no device is in the Error state.
   * @return True if no device is in the Error state.
   */
  bool noDeviceIsInErrorState() const;

  /*!
   * Check if no device is in the Fatal state.
   * @return True if no device is in the Fatal state.
   */
  bool noDeviceIsInFatalState() const;

  /*!
   * Check if all devices are within the SDK joint position limits.
   * @return True if all devices are within the SDK joint position limits.
   */
  bool allDevicesAreWithinJointPositionLimitsSdk() const;

  /*!
   * Check if all devices are within the soft joint position limits.
   * @return True if all devices are within the soft joint position limits.
   */
  bool allDevicesAreWithinJointPositionLimitsSoft() const;

  /*!
   * Check if all devices are within the hard joint position limits.
   * @return True if all devices are within the hard joint position limits.
   */
  bool allDevicesAreWithinJointPositionLimitsHard() const;

  //@}

  /*!
   * @name Flash storage
   *
   * Methods to control the flash storage on the devices.
   */
  //@{

  /*!
   * Calls Anydrive::eraseFlashStorage for all devices.
   * @return True if successful.
   */
  bool eraseFlashStorage();

  /*!
   * Calls Anydrive::resetFlashStorageSections for all devices.
   * @param flashStorageSection Bit-field indicating the flash storage sections to reset.
   * @return True if successful.
   */
  bool resetFlashStorageSections(const uint8_t flashStorageSections);

  //@}

  /*!
   * @name Calibration
   *
   * Methods to interface with the device calibrations.
   */
  //@{

  /*!
   * Run a calibration for a given device.
   * @param deviceName                                Name of the device to run the calibration.
   * @param calibrationModeEnum                       Mode of the calibration.
   * @param gearAndJointEncoderHomingAbsolute         In case of the gear and joint encoder homing calibration, this flag indicates whether
   * it shall be done absolute or relative.
   * @param gearAndJointEncoderHomingNewJointPosition In case of the gear and joint encoder homing calibration, this is the new joint
   * position.
   */
  virtual bool calibrate(const std::string& deviceName, const calibration::CalibrationModeEnum calibrationModeEnum,
                         const bool gearAndJointEncoderHomingAbsolute, const double gearAndJointEncoderHomingNewJointPosition);

  /*!
   * Calls Anydrive::resetCustomCalibrationsToFactory for all devices.
   * @param calibrationState Select which calibration(s) should be reset.
   * @return True if successful.
   */
  bool resetCustomCalibrationsToFactory(const calibration::CalibrationState calibrationState);

  /*!
   * Calls Anydrive::writeFactoryCalibration for all devices.
   * @return True if successful.
   */
  bool writeFactoryCalibration();

  //@}

  /*!
   * @name Configuration
   *
   * Methods to interface with the device configurations.
   */
  //@{

  /*!
   * Calls Anydrive::setMaxCurrent for all drives.
   * @param maxCurrent The maximal current.
   * @return True if successful.
   */
  bool setMaxCurrent(const double maxCurrent);

  /*!
   * Calls Anydrive::setMaxMotorVelocity for all drives.
   * @param maxMotorVelocity The maximal motor velocity.
   * @return True if successful.
   */
  bool setMaxMotorVelocity(const double maxMotorVelocity);

  /*!
   * Calls Anydrive::setMaxJointTorque for all drives.
   * @param maxJointTorque The maximal joint torque.
   * @return True if successful.
   */
  bool setMaxJointTorque(const double maxJointTorque);

  /*!
   * Calls Anydrive::setCurrentIntegratorSaturation for all drives.
   * @param saturation The integrator saturation for the current controller.
   * @return True if successful.
   */
  bool setCurrentIntegratorSaturation(const double saturation);

  /*!
   * Calls Anydrive::setJointTorqueIntegratorSaturation for all drives.
   * @param saturation The integrator saturation for the joint torque controller.
   * @return True if successful.
   */
  bool setJointTorqueIntegratorSaturation(const double saturation);

  /*!
   * Calls Anydrive::setDirection for all drives.
   * @param direction The rotation direction.
   * @return True if successful.
   */
  bool setDirection(const int16_t direction);

  /*!
   * Calls Anydrive::setJointPositionLimitsSdk for all drives.
   * @param limits SDK joint positions limits.
   * @return True if successful.
   */
  bool setJointPositionLimitsSdk(const common::Limits& limits);

  /*!
   * Calls Anydrive::setJointPositionLimitsSoft for all drives.
   * @param limits Soft joint positions limits.
   * @return True if successful.
   */
  bool setJointPositionLimitsSoft(const common::Limits& limits);

  /*!
   * Calls Anydrive::setJointPositionLimitsHard for all drives.
   * @param limits Hard joint positions limits.
   * @return True if successful.
   */
  bool setJointPositionLimitsHard(const common::Limits& limits);

  /*!
   * Calls Anydrive::setControlGains for all drives.
   * @param mode     Control mode to set the gains for.
   * @param pidGains Control gains.
   * @return True if successful.
   */
  bool setControlGains(const mode::ModeEnum mode, const mode::PidGainsF& pidGains);

  /*!
   * Calls Anydrive::setErrorStateBehavior for all drives.
   * @param errorStateBehavior Behavior of the device.
   * @return True if successful.
   */
  bool setErrorStateBehavior(const uint16_t errorStateBehavior);

  /*!
   * Calls Anydrive::writeConfiguration for all drives.
   * @return True if successful.
   */
  bool writeConfiguration();

  //@}

  /*!
   * @name Control
   *
   * Methods to run controllers on the devices.
   */
  //@{

  /*!
   * Calls Anydrive::sendControlwords for all drives.
   * @param controlwordId ID of the controlword to send.
   */
  void sendControlwords(const uint16_t controlwordId);

  /*!
   * Calls Anydrive::stageDisable for all drives.
   */
  void stageDisables();

  /*!
   * Calls Anydrive::stageFreezes for all drives.
   */
  void stageFreezes();

  /*!
   * Calls Anydrive::stageZeroJointTorques for all drives.
   */
  void stageZeroJointTorques();

  /*!
   * Calls Anydrive::stageCommand for all drives.
   * @param commands Commands to stage for each ANYdrive. Order and size must match the the list of ANYdrives.
   */
  void stageCommands(const std::vector<Command>& commands);

  /*!
   * Start achieving a joint position configuration with all devices.
   * @param configurationName Name of the configuration to be achieved.
   * @return True if commands were generated successfully.
   */
  bool achieveJointPositionConfiguration(const std::string& configurationName);

  /*!
   * Check if joint position configuration has been reached.
   * @return True if all devices reached the joint position.
   */
  bool achieveJointPositionConfigurationIsActive();

  /*!
   * Abort achieving the joint position configuration.
   */
  void abortAchieveJointPositionConfiguration();

  //@}

  /*!
   * @name Readings
   *
   * Methods to access the readings of the devices.
   */
  //@{

  /*!
   * Get the newest readings which have been received.
   * @return Newest readings.
   */
  std::vector<ReadingExtended> getReadings() const;

  /*!
   * Get the newest readings which have been received.
   * @param readings Return argument, will contain the newest readings.
   */
  void getReadings(std::vector<ReadingExtended>& readings) const;

  //@}

  /*!
   * @name Debugging
   *
   * Methods for debugging.
   */
  //@{

  /*!
   * Print ANYdrive infos for debugging.
   */
  void printInfo() const;

  //@}

  /*!
   * Request the shutdown of the ANYdrive Manager.
   */
  void requestShutdown();

 protected:
  /*!
   * Create a new ANYdrive.
   * @return New ANYdrive.
   */
  virtual AnydrivePtr createAnydrive() const;

  /*!
   * Worker updating the ANYdrive Manager if standalone is true.
   * @param event Worker event.
   * @return True if successful.
   */
  bool updateWorkerCb(const any_worker::WorkerEvent& event);

  /*!
   * Signal callback function.
   * @param signum Signal code.
   */
  void handleSignal(const int signum);
};

using AnydriveManagerPtr = std::shared_ptr<AnydriveManager>;

}  // namespace anydrive
