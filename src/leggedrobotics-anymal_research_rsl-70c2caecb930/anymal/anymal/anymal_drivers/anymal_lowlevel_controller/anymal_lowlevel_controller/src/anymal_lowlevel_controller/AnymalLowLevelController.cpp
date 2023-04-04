// any measurements ros
#include <any_measurements_ros/ConvertRosMessages.hpp>

// anymal model ros
#include <anymal_model_ros/conversions.hpp>

// series elastic actuator anydrive
#include <series_elastic_actuator_anydrive/series_elastic_actuator_anydrive.hpp>

// ros
#include <ros/package.h>

// anymal lowlevel controller
#include "anymal_lowlevel_controller/AnymalLowLevelController.hpp"


namespace anymal_lowlevel_controller {


AnymalLowLevelController::AnymalLowLevelController(NodeHandlePtr nh)
: any_node::Node(nh),
  freezesAreSet_(false),
  hasDysfunction_(false),
  hasDysfunctionPublished_(false),
  timePrevIteration_(std::chrono::steady_clock::now()),
  iterDurationMs_(0.0),
  updateDurationMs_(0.0),
  stopUpdating_(false),
  updateCounter_(1u),
  receiveMaxLockTime_{std::chrono::microseconds{50}},
  sendMaxLockTime_{std::chrono::microseconds{50}},
  logFileTypes_({signal_logger::LogFileType::BINARY}) {}

bool AnymalLowLevelController::init()
{
  try {
    double timeout = actuatorCommandTimeout_.toSec();
    param_io::getParam(getNodeHandle(), "time_step", timeStep_);
    param_io::getParam(getNodeHandle(), "run_publishers", runPublishers_);
    param_io::getParam(getNodeHandle(), "actuator_command_timeout", timeout);
    actuatorCommandTimeout_.fromSec(timeout);

    // System.
    system_.reset(new System());
    if (!system_->init(getNodeHandle(), timeStep_)) {
      return false;
    }

    // State machine.
    stateMachine_.reset(new state_machine::StateMachine(system_));

    // ROS interface.
    softEmergencyStopSubscriber_ = subscribe("soft_emergency_stop", "soft_emergency_stop", 10, &AnymalLowLevelController::softEmergencyStopCb, this, ros::TransportHints().tcpNoDelay());
    hardEmergencyStopSubscriber_ = subscribe("hard_emergency_stop", "hard_emergency_stop", 10, &AnymalLowLevelController::hardEmergencyStopCb, this, ros::TransportHints().tcpNoDelay());
    softEmergencyStopPublisher_ = advertise<std_msgs::Empty>("soft_emergency_stop", "soft_emergency_stop", 1, false);

    auto optionsActuatorCommands = std::make_shared<cosmo_ros::SubscriberRosOptions<ActuatorCommandsShm>>("actuator_commands", std::bind(&AnymalLowLevelController::actuatorCommandsCallback, this, std::placeholders::_1), getNodeHandle());
    optionsActuatorCommands->autoSubscribe_ = false;
    optionsActuatorCommands->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
    optionsActuatorCommands->tryRosResubscribing_ = false;

    actuatorCommandsSubscriber_ =
        cosmo_ros::subscribeShmRos<ActuatorCommandsShm,
                                   ActuatorCommandsRos,
                                   anymal_model_ros::conversion_traits::ConversionTraits>("actuator_commands", optionsActuatorCommands);

    auto optionsActuatorReadings = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_readings", getNodeHandle());
    optionsActuatorReadings->autoPublishRos_ = false;
    optionsActuatorReadings->rosQueueSize_ = 10u;
    optionsActuatorReadings->rosLatch_ = false;
    actuatorReadingsPublisher_ =
        cosmo_ros::advertiseShmRos<ActuatorReadingsShm,
                                   ActuatorReadingsRos,
                                   anymal_model_ros::conversion_traits::ConversionTraits>("actuator_readings", optionsActuatorReadings);

    // Setup signal logger
    bool logExtendedReadings_= param<bool>("log_extended_readings", false);
    bool logElectricalPhaseReadings_= param<bool>("log_elec_phase_readings", false);

    std::string loggingScriptFilename = param<std::string>("logger/config_file", "");
    if (loggingScriptFilename.empty()){
      loggingScriptFilename = ros::package::getPath("anymal_lowlevel_controller") + std::string{"/config/logging.yaml"};
    }
    double loggerSamplingWindow = param<double>("logger/sampling_window", 10.0);

    std::string logFileType = param<std::string>("logger/logfile_type", "binary");
    if(logFileType == "csv"){
    	logFileTypes_ = {signal_logger::LogFileType::CSV};
    } else if(logFileType == "bag") {
    	logFileTypes_ = {signal_logger::LogFileType::BAG};
    } else {
    	logFileTypes_ = {signal_logger::LogFileType::BINARY};
    }

    signal_logger::setSignalLoggerRos(&getNodeHandle());
    signal_logger::SignalLoggerOptions siloOptions;
    siloOptions.loggerName_ = "lowLevelController";
    siloOptions.updateFrequency_ = static_cast<int>(1.0/timeStep_);
    siloOptions.maxLoggingTime_ = loggerSamplingWindow;
    siloOptions.collectScriptFileName_ = loggingScriptFilename;
    signal_logger::logger->initLogger(siloOptions);
    signal_logger::add(iterDurationMs_, std::string{"LLControllerIterDurationMs"});
    signal_logger::add(updateDurationMs_, std::string{"LLControllerUpdateDurationMs"});
    signal_logger::add(actuatorCommandsMissCount_, std::string{"LLActuatorCommandsMissCount"});
    signal_logger::add(workingCounterTooLowCount_, std::string{"LLWorkingCounterTooLowCount"});

    if (logExtendedReadings_) {
      //Add readings logging
      std::string ns { "/readingsExtended/" };
      signal_logger::LogElementAction action = signal_logger::LogElementAction::SAVE_AND_PUBLISH;
      const double maxBufferSize = siloOptions.maxLoggingTime_ * siloOptions.updateFrequency_; //TODO: Remove once SiLo generates default buffer size this way.
      for (const auto& actuatorKey : AD::getActuatorKeys()) {
        const auto actuatorEnum = actuatorKey.getEnum();
        const auto actuatorName = actuatorKey.getName();
        auto& readingExtended = actuatorReadingsExtended_[actuatorEnum];

        signal_logger::add(readingExtended.getState().getJointPosition(), std::string { "jointPos_" } + actuatorName, ns, "rad", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getJointVelocity(), std::string { "jointVel_" } + actuatorName, ns, "rad/s", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getJointTorque(), std::string { "jointTor_" } + actuatorName, ns, "Nm", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getJointAcceleration(), std::string { "jointAcc_" } + actuatorName, ns, "rad/s2", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getGearPosition(), std::string { "gearPos_" } + actuatorName, ns, "rad", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getGearVelocity(), std::string { "gearVel_" } + actuatorName, ns, "rad/s", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getMotorPosition(), std::string { "motorPos_" } + actuatorName, ns, "rad", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getMotorVelocity(), std::string { "motorVel_" } + actuatorName, ns, "rad/s", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getTemperature(), std::string { "temperature_" } + actuatorName, ns, "C", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getCurrent(), std::string { "current_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
        signal_logger::add(readingExtended.getState().getVoltage(), std::string { "voltage_" } + actuatorName, ns, "V", 1, action, maxBufferSize);

        if (logElectricalPhaseReadings_) {
          signal_logger::add(readingExtended.getState().getDesiredCurrentD(), std::string { "desiredCurrentD_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredCurrentD(), std::string { "measuredCurrentD_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getDesiredCurrentQ(), std::string { "desiredCurrentQ_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredCurrentQ(), std::string { "measuredCurrentQ_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredCurrentPhaseU(), std::string { "measuredCurrentPhaseU_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredCurrentPhaseV(), std::string { "measuredCurrentPhaseV_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredCurrentPhaseW(), std::string { "measuredCurrentPhaseW_" } + actuatorName, ns, "A", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredVoltagePhaseU(), std::string { "measuredVoltagePhaseU_" } + actuatorName, ns, "V", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredVoltagePhaseV(), std::string { "measuredVoltagePhaseV_" } + actuatorName, ns, "V", 1, action, maxBufferSize);
          signal_logger::add(readingExtended.getState().getMeasuredVoltagePhaseW(), std::string { "measuredVoltagePhaseW_" } + actuatorName, ns, "V", 1, action, maxBufferSize);
        }
      }
    }

    // Start logger
    signal_logger::logger->updateLogger();
    signal_logger::logger->startLogger();

    if (runPublishers_) {
      activeStatePublisher_ = advertise<anymal_msgs::AnymalLowLevelControllerState>("active_state", "active_state", 1, true);
      batteryVoltagePublisher_ = advertise<std_msgs::Float32>("battery_voltage", "battery_voltage", 1, true);
      dysfunctionPublisher_ = advertise<any_msgs::State>("dysfunction", "dysfunction", 1, true);

      // Publish dysfunction once.
      publishDysfunctionState();

      any_worker::WorkerOptions pubOptions;
      pubOptions.callback_ = std::bind(&AnymalLowLevelController::publishWorker, this, std::placeholders::_1);
      pubOptions.defaultPriority_ = 0; // this has low priority
      pubOptions.name_ = "AnymalLowLevelController::publisherWorker";
      pubOptions.timeStep_ = std::numeric_limits<double>::infinity();
      if (!this->addWorker(pubOptions)) {
        MELO_ERROR_STREAM("[AnymalLowLevelController] Worker " << pubOptions.name_ << "could not be added!");
        return false;
      }

    }
    goToStateServer_ = advertiseService("go_to_state", "go_to_state", &AnymalLowLevelController::goToStateCb, this);
    enableActuatorComService_ = advertiseService("toggle_actuator_communication", "toggle_actuator_communication", &AnymalLowLevelController::toggleActuatorCommunication, this);

    // Start sending sync signal
    syncMaster_ = std::unique_ptr<cosmo::SyncMaster>(new cosmo::SyncMaster("anymal_sync", timeStep_, 10, true));
    syncMaster_->start();

    any_worker::WorkerOptions options;
    options.callback_ = std::bind(&AnymalLowLevelController::updateController, this, std::placeholders::_1);
    options.defaultPriority_ = 10; // this has high priority
    options.name_ = "AnymalLowLevelController::updateController";
    options.timeStep_ = std::numeric_limits<double>::infinity();
    if (!this->addWorker(options)) {
      MELO_ERROR_STREAM("[AnymalLowLevelController] Worker " << options.name_ << "could not be added!");
      return false;
    }

    MELO_INFO_STREAM("[AnymalLowLevelController]: Initialized successfully.");

  }
  catch (message_logger::log::melo_fatal& exception) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught a message logger exception: " << exception.what());
    return false;
  }
  catch (std::exception& exception) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught a standard exception: " << exception.what());
    return false;
  }
  catch (...) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught an unspecified exception.");
    return false;
  }

  return true;
}

void AnymalLowLevelController::preCleanup()
{
  stopUpdating_ = true;
  cvUpdate_.notify_all();
  syncMaster_->stop(true);

  // System
  system_->preCleanup();
}

void AnymalLowLevelController::cleanup()
{
  try {
    // System
    system_->cleanup();

    // ROS interface.
    softEmergencyStopSubscriber_.shutdown();
    hardEmergencyStopSubscriber_.shutdown();
    softEmergencyStopPublisher_.shutdown();
    enableActuatorComService_.shutdown();
    if (runPublishers_)
    {
      activeStatePublisher_.shutdown();
      batteryVoltagePublisher_.shutdown();
      dysfunctionPublisher_.shutdown();
    }
    goToStateServer_.shutdown();

    signal_logger::logger->cleanup();

    MELO_INFO_STREAM("[AnymalLowLevelController]: Cleaned up successfully.");
  }
  catch (message_logger::log::melo_fatal& exception) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught a message logger exception: " << exception.what());
  }
  catch (std::exception& exception) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught a standard exception: " << exception.what());
  }
  catch (...) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught an unspecified exception.");
  }
}

bool AnymalLowLevelController::updateController(const any_worker::WorkerEvent& event)
{
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  // std::chrono::time_point<std::chrono::steady_clock> startSub, endSub;
  int64_t elapsedSub;

  const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
  const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

  while (!stopUpdating_) {
    // Waiting for sync msg from simulation or lowlevel ctrl
    auto msg = syncMaster_->waitForSync();

    // Stop immediately if notified.
    if (stopUpdating_) {
      return true;
    }

    auto timeCurrIteration = std::chrono::steady_clock::now();
    const int64_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(timeCurrIteration - timePrevIteration_).count();
    iterDurationMs_ = (double)elapsed*1e-6;
    timePrevIteration_ = timeCurrIteration;

    //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();

    try {
      // startSub = std::chrono::steady_clock::now();
      updateCommands();
      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval1_ = (double)elapsedSub*1e-6;
      // startSub = std::chrono::steady_clock::now();

      if (system_->isActuatorSetup() && system_->isActutatorCommunicationEnabled()) {
#ifndef ANYDRIVE1X
        system_->getAnydriveManager()->updateCommunicationManagerReadMessages();
#endif
        system_->getAnydriveManager()->updateProcessReadings();

        // endSub = std::chrono::steady_clock::now();
        // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
        // durationSubinterval2_ = (double)elapsedSub*1e-6;
        // startSub = std::chrono::steady_clock::now();

        system_->getAnydriveManager()->updateStageCommands();
      }
      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval3_ = (double)elapsedSub*1e-6;
      // startSub = std::chrono::steady_clock::now();

      system_->updateSensors(event);

      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval4_ = (double)elapsedSub*1e-6;
      // startSub = std::chrono::steady_clock::now();

      updateActiveSubstates();

      if (!stateMachine_->checkActiveConditions()) {
        stateMachine_->setGoalStateEnum(state_machine::StateEnum::StateIdle);
      }

      // If one ANYdrive is in fatal state, go to anymal lowlevel controller fatal state
      if (system_->getAnydrivesStateEnum() == state_machine::AnydrivesStateEnum::Fatal) {
        stateMachine_->setGoalStateEnum(state_machine::StateEnum::StateFatal);
      }

      if (system_->getAnydrivesStateEnum() == state_machine::AnydrivesStateEnum::Missing) {
        stateMachine_->setGoalStateEnum(state_machine::StateEnum::ActionConnectActuators);
      }

      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval5_ = (double)elapsedSub*1e-6;
      // startSub = std::chrono::steady_clock::now();

      stateMachine_->update();

      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval6_ = (double)elapsedSub*1e-6;
      // startSub = std::chrono::steady_clock::now();

      if (system_->isActuatorSetup() && system_->isActutatorCommunicationEnabled()) {
        system_->getAnydriveManager()->updateSendStagedCommands();
        system_->getAnydriveManager()->updateCommunicationManagerWriteMessages();
      }
      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval7_ = (double)elapsedSub*1e-6;
      // startSub = std::chrono::steady_clock::now();

      updateMeasurements();

      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval8_ = (double)elapsedSub*1e-6;
      // startSub = std::chrono::steady_clock::now();

      // Collect data for logger
      signal_logger::logger->collectLoggerData();

      // endSub = std::chrono::steady_clock::now();
      // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
      // durationSubinterval9_ = (double)elapsedSub*1e-6;

      // Notify the publish worker
      updateCounter_++;
      cvUpdate_.notify_all();
    }
    catch (message_logger::log::melo_fatal& exception) {
      MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught a message logger exception: " << exception.what());
      return false;
    }
    catch (std::exception& exception) {
      MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught a standard exception: " << exception.what());
      return false;
    }
    catch (...) {
      MELO_ERROR_STREAM("[AnymalLowLevelController]: Caught an unspecified exception.");
      return false;
    }


    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    const int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    updateDurationMs_ = (double)elapsedTimeNSecs*1e-6;
    if (elapsedTimeNSecs > timeStepNSecs) {
      MELO_WARN_THROTTLE(1.0, "Computation of low-level controller is not real-time! Computation took: %lf ms. (Warning is throttled: 1s)", (double)elapsedTimeNSecs*1e-6);
    }
  }
  return true;
}


void AnymalLowLevelController::notifyControllerStatus(const bool running)
{
  if (running) {
    MELO_INFO("[AnymalLowLevelController]: Highlevel controller started.");
  }
  else {
    MELO_INFO("[AnymalLowLevelController]: Highlevel controller terminated or timed out.");
  }
}


void AnymalLowLevelController::updateCommands()
{
  if (!system_->isActuatorSetup() || system_->getAnydrivesStateEnum() == state_machine::AnydrivesStateEnum::Missing) {
    return;
  }

  auto now = ros::Time::now();
  if (actuatorCommandsSubscriber_->receive(receiveMaxLockTime_)) {
    lastActuatorCommandTimestamp_ = now;
  } else {
    ++actuatorCommandsMissCount_;

    if (!lastActuatorCommandTimestamp_.isZero() && now - lastActuatorCommandTimestamp_ > actuatorCommandTimeout_) {
      // go to idle state to freeze the drives
      stateMachine_->setGoalStateEnum(state_machine::StateEnum::StateIdle);
      MELO_ERROR_THROTTLE(1, "Did not receive actuator commands for %d iterations. Is the highlevel controller running?",
                          actuatorCommandsMissCount_);
    }
  }

  // Check if actuator commands has the right size.
  if (actuatorCommands_.size() != AD::getActuatorsDimension()) {
    MELO_ERROR_STREAM("Actuator commands has the wrong size. Actual size: " << actuatorCommands_.size() << " vs. expected size: "
                                                                            << AD::getActuatorsDimension() << ".")
    return;
  }

  bool freezesAreSet = true;

  for(auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    freezesAreSet &= (actuatorCommands_[actuatorEnum].getModeEnum() == series_elastic_actuator::SeActuatorCommand::MODE_FREEZE);
  }
  freezesAreSet_ = freezesAreSet;

  // Safety: Only accept commands when in state machine is in state Operational.
  if (stateMachine_->getActiveStateEnum() != state_machine::StateEnum::StateOperational)
    return;

  for(auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    if (system_->getAnydrive(actuatorEnum)) {
      anydrive::Command command;
      series_elastic_actuator_anydrive::seActuatorToAnydrive(command, actuatorCommands_[actuatorEnum]);
      system_->getAnydrive(actuatorEnum)->stageCommand(command);
    }
  }
}

void AnymalLowLevelController::updateActiveSubstates()
{
  stateMachine_->setActiveSubstates(system_->getActiveSubstates());
}

void AnymalLowLevelController::updateMeasurements()
{
  if (!system_->isActuatorSetup() || system_->getAnydrivesStateEnum() == state_machine::AnydrivesStateEnum::Missing) {
    return;
  }

  // std::chrono::time_point<std::chrono::steady_clock> startSub, endSub;
  // int64_t elapsedSub;

  // Actuators.
  bool isOk = true;
  const ros::Time rosStamp = ros::Time::now();
  const any_measurements::Time stamp  = any_measurements_ros::fromRos(rosStamp);
  for(auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    if (!system_->getAnydrive(actuatorEnum)) {
      continue;
    }
    series_elastic_actuator::SeActuatorReadingExtended readingExtended;
    series_elastic_actuator_anydrive::anydriveToSeActuator(readingExtended, system_->getAnydrive(actuatorEnum)->getReading());
    actuatorReadings_[actuatorEnum].setState(static_cast<const series_elastic_actuator::SeActuatorState&>(readingExtended.getState()));
    actuatorReadings_[actuatorEnum].setCommanded(readingExtended.getCommanded());
    actuatorReadings_[actuatorEnum].getState().setStamp(stamp);
    actuatorReadingsExtended_[actuatorEnum].setState(readingExtended.getState());
    actuatorReadingsExtended_[actuatorEnum].setCommanded(readingExtended.getCommanded());
    actuatorReadingsExtended_[actuatorEnum].getState().setStamp(stamp);

#ifdef ANYDRIVE1X
    isOk &= !system_->getAnydrive(actuatorEnum)->getStatusword().hasOverTemperatureWarningDriver();
#else
    isOk &= !system_->getAnydrive(actuatorEnum)->getStatusword().hasWarningHighTemperatureBridge();
    isOk &= !system_->getAnydrive(actuatorEnum)->getStatusword().hasWarningHighTemperatureStator();
    isOk &= !system_->getAnydrive(actuatorEnum)->getStatusword().hasWarningHighTemperatureCpu();
#endif
  }

  hasDysfunction_ = !isOk;

  // startSub = std::chrono::steady_clock::now();

  if(!actuatorReadingsPublisher_->publish(actuatorReadings_, sendMaxLockTime_)) {
    ++actuatorReadingsMissCount_;
  }

  // endSub = std::chrono::steady_clock::now();
  // elapsedSub = std::chrono::duration_cast<std::chrono::nanoseconds>(endSub - startSub).count();
  // durationSubinterval10_ = (double)elapsedSub*1e-6;
}

bool AnymalLowLevelController::goToStateCb(GoToState::Request& req, GoToState::Response& res)
{
  const state_machine::StateEnum stateEnum = state_machine::stateMsgToEnum(req.state);
  if (stateEnum == stateMachine_->getActiveStateEnum()) {
    return true;
  }

  if (state_machine::isAction(stateMachine_->getActiveStateEnum()) && state_machine::isAction(stateEnum)) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Cannot enter action when an action is running.");
    return false;
  }

  if (stateEnum == state_machine::StateEnum::StateOperational && !freezesAreSet_) {
    MELO_ERROR_STREAM("[AnymalLowLevelController]: Cannot enter StateOperational when controller is running. Sending soft emergency stop.");
    softEmergencyStopPublisher_.publish(std_msgs::Empty());
    return false;
  }

  if (stateEnum == state_machine::StateEnum::ActionActuatorsDisable) {
    MELO_INFO("Actuators have been disabled. Requesting soft emergency stop.")
    softEmergencyStopPublisher_.publish(std_msgs::Empty());
  }

  stateMachine_->setGoalStateEnum(state_machine::stateMsgToEnum(req.state));
  return true;
}

void AnymalLowLevelController::softEmergencyStopCb(const std_msgs::Empty::ConstPtr& msg)
{
  MELO_WARN("[AnymalLowLevelController]: Received soft emergency stop.");
  signal_logger::logger->saveLoggerData(logFileTypes_);
  if (stateMachine_->getActiveStateEnum() != state_machine::StateEnum::StateOperational) {
    stateMachine_->setGoalStateEnum(state_machine::StateEnum::StateIdle);
  }
}

void AnymalLowLevelController::hardEmergencyStopCb(const std_msgs::Empty::ConstPtr& msg)
{
  MELO_WARN("[AnymalLowLevelController]: Received hard emergency stop.");
  //stateMachine_->setGoalStateEnum(state_machine::StateEnum::StateIdle);
  //system_->enableActuatorCommunication(false);
}

bool AnymalLowLevelController::toggleActuatorCommunication(any_msgs::Toggle::Request& req, any_msgs::Toggle::Response& res)
{
  system_->enableActuatorCommunication(static_cast<bool>(req.enable));
  res.success = true;
  return true;
}

void AnymalLowLevelController::actuatorCommandsCallback(const ActuatorCommandsShm& msg)
{
  actuatorCommands_ = msg;
}

bool AnymalLowLevelController::publishWorker(const any_worker::WorkerEvent& workerEvent)
{
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 0lu;

  while(!stopUpdating_) {
    /* Suspend the thread until new anymal state message has arrived.
    * We use the conditional variable for that, but we need to check the counter
    * to handle spurious wake-ups.
    */
    boost::unique_lock<boost::mutex> lock{mutexPublishUpdate_};
    cvUpdate_.wait(lock,[this,localCounter](){
           if (stopUpdating_) return true;
           return (updateCounter_ > localCounter);
    });
    localCounter = updateCounter_;

    // Stop immediately
    if (stopUpdating_) {
      return true;
    }

    if (!system_->isActuatorSetup() || system_->getAnydrivesStateEnum() == state_machine::AnydrivesStateEnum::Missing) {
      continue;
    }

    //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();

    actuatorReadingsPublisher_->sendRos();
    system_->getAnydriveManager()->sendRos();

    // Publish active state if it has changed.
    const state_machine::StateEnum activeStateEnum = stateMachine_->getActiveStateEnum();
    if ((activeStatePublisher_.getNumSubscribers() > 0u ||
         activeStatePublisher_.isLatched()) &&
        activeStateEnum != lastActiveStateEnum_) {
      activeStatePublisher_.publish(state_machine::stateEnumToMsg(activeStateEnum));
      lastActiveStateEnum_ = activeStateEnum;
    }

    if ((batteryVoltagePublisher_.getNumSubscribers() > 0u ||
         batteryVoltagePublisher_.isLatched()) &&
        ++publishBatteryVoltageCounter_ == publishBatteryVoltageDecimation_) {
      // Publish battery voltage with a certain decimation rate.
      const double alpha = 0.2;
      const double batteryVoltageReading = system_->getAnydrive(AD::ActuatorEnum::LF_HAA) ? system_->getAnydrive(AD::ActuatorEnum::LF_HAA)->getReading().getState().getVoltage() : 0.0;
      if (!std::isnan(batteryVoltageReading)) {
        if (batteryVoltage_ == 0.0) {
          // Initialize the voltage.
          batteryVoltage_ = batteryVoltageReading;
        }
        else {
          // Update the voltage.
          batteryVoltage_ = (1.0 - alpha)*batteryVoltage_ + alpha*batteryVoltageReading;
        }
        std_msgs::Float32 batteryVoltageMsg;
        batteryVoltageMsg.data = batteryVoltage_;
        batteryVoltagePublisher_.publish(batteryVoltageMsg);
      }
      publishBatteryVoltageCounter_ = 0;
    }

    // Update working counter too low count.
    workingCounterTooLowCount_ = system_->getAnydriveManager()->getWorkingCounterTooLowCount();

    // Publish dysfunction if the state has changed.
    if (hasDysfunction_ != hasDysfunctionPublished_) {
      publishDysfunctionState();
    }

    // Publish signal logger data.
    signal_logger::logger->publishData();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
    const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

    if (elapsedTimeNSecs > timeStepNSecs) {
       MELO_WARN("Computation of publish worker of Low-level Controller is not real-time! Computation time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
  }

  return true;
}

bool AnymalLowLevelController::signalLoggerWorker(const any_worker::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 0lu;

  while (!stopUpdating_) {
    /* Suspend the thread until new anymal state message has arrived.
    * We use the conditional variable for that, but we need to check the counter
    * to handle spurious wake-ups.
    */
    boost::unique_lock<boost::mutex> lock{mutexSignalLoggerUpdate_};
    cvUpdate_.wait(lock,[this,localCounter](){
           if (stopUpdating_) return true;
           return (updateCounter_ > localCounter);
    });
    localCounter = updateCounter_;

    // Stop immediately
    if (stopUpdating_) {
      return true;
    }

    //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();

    // Dot the work
    signal_logger::logger->publishData();


    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
    const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

    if (elapsedTimeNSecs > timeStepNSecs) {
       MELO_WARN("Computation of signal logger of Low-level Controller is not real-time! Computation time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
  }

  return true;
}

void AnymalLowLevelController::publishDysfunctionState()
{
  if (dysfunctionPublisher_.getNumSubscribers() > 0u &&
      !dysfunctionPublisher_.isLatched()) {
    return;
  }

  any_msgs::State state;
  state.stamp = ros::Time::now();
  state.is_ok = !hasDysfunction_;
  dysfunctionPublisher_.publish(state);

  // Remember the state that has been published, in order to publish state changes only.
  hasDysfunctionPublished_ = hasDysfunction_.load();
}

} // anymal_lowlevel_controller
