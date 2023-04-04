/*!
 * @file    AnymalHighLevelController.cpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */


#include "anymal_highlevel_controller/AnymalHighLevelController.hpp"

#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <message_logger/message_logger.hpp>
#include <signal_logger/signal_logger.hpp>
#include <parameter_handler/parameter_handler.hpp>
#include <parameter_handler_ros/ParameterHandlerRos.hpp>
#include <anymal_model_ros/conversions.hpp>
#include <anymal_model_ros/initializations.hpp>
#include <anymal_model_ros/inertial_parameters_ros.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <csignal>
#include <iostream>


#define NOTIFICATION_ID "LMC"

namespace anymal_highlevel_controller {

AnymalHighLevelController::AnymalHighLevelController(any_node::Node::NodeHandlePtr nh):
    any_node::Node(nh),
    isRealRobot_(true),
    timeStep_(0.0025),
    model_(),
    controllerManager_("anymal_roco::RocoState", "anymal_roco::RocoCommand"),
    poseCommandsOutdated_(true),
    velocityCommandsOutdated_(true),
    commandsTimeOut_(0.5),
    timePrevIteration_(std::chrono::steady_clock::now()),
    iterDurationMs_(0.0),
    updateDurationMs_(0.0),
    receiveMaxLockTime_{std::chrono::microseconds{50}},
    sendMaxLockTime_{std::chrono::microseconds{50}}
{
  stopUpdating_ = false;
  updateCounter_ = 01u;
}

bool AnymalHighLevelController::init() {
  timeStep_ = param<double>("time_step", 0.0025);
  updateDecimation_ = param<uint32_t>("update_decimation", 1u);
  updateCounterDecimation_ = 0u;
  timeStep_ *= updateDecimation_;
  MELO_INFO("[AnymalHighLevelController::init] time step: %lf (decimation: %d)", timeStep_, updateDecimation_);

  //--- Read parameters.
  bool simulation = param<bool>("controller/simulation", false);
  isRealRobot_ = !simulation;
  MELO_INFO_STREAM("[AnymalHighLevelController] Is in simulation: " << (simulation ? "yes": "no"));

  //--- Configure logger.
  std::string loggingScriptFilename = param<std::string>("logger/config_file", "");

  if (loggingScriptFilename.empty() || !fileHasContent(loggingScriptFilename)){
    auto defaultLoggingScriptFilename = ros::package::getPath("anymal_highlevel_controller") + std::string{"/config/logging.yaml"};
    std::ifstream  srcFile(defaultLoggingScriptFilename.c_str(), std::ios::binary);
    std::ofstream  dstFile(loggingScriptFilename.c_str(), std::ios::binary);
    dstFile << srcFile.rdbuf();
  }
  double loggerSamplingWindow = param<double>("logger/sampling_window", 60.0);

  std::string loggerType = param<std::string>("logger/type", "none");

  if (loggerType.compare("ros") == 0) {
    MELO_INFO("[AnymalHighLevelController::init] Logger type: ros");
    // initialize ros logger
    signal_logger::setSignalLoggerRos(&getNodeHandle());
  }
  else if (loggerType.compare("std") == 0) {
    MELO_INFO("[AnymalHighLevelController::init] Logger type: std");
    // initialize std logger as fallback logger
    signal_logger::setSignalLoggerStd();
  }
  else {
    MELO_INFO("[AnymalHighLevelController::init] Logger type: none");
    signal_logger::setSignalLoggerNone();
  }

  signal_logger::SignalLoggerOptions siloOptions;
  siloOptions.loggerName_ = "highLevelController";
  siloOptions.updateFrequency_ = static_cast<int>(1.0/timeStep_);
  siloOptions.maxLoggingTime_ = loggerSamplingWindow;
  siloOptions.collectScriptFileName_ = loggingScriptFilename;
  signal_logger::logger->initLogger(siloOptions);

  MELO_INFO("[AnymalHighLevelController::init] Initialize logger with sampling window: %4.2fs, "
                ", script: %s .", siloOptions.maxLoggingTime_, siloOptions.collectScriptFileName_.c_str());

  //--- Configure parameter handler
  parameter_handler::handler.reset(new parameter_handler_ros::ParameterHandlerRos());
  parameter_handler_ros::ParameterHandlerRos* parameterHandlerRos = static_cast<parameter_handler_ros::ParameterHandlerRos*>(parameter_handler::handler.get());
  parameterHandlerRos->setNodeHandle(&getNodeHandle());
  parameterHandlerRos->initializeServices();
  //---

  // manager logger options
  rocoma::LoggerOptions managerLoggerOptions;
  managerLoggerOptions.enable = param<bool>("logger/enable", false);
  managerLoggerOptions.updateOnStart = param<bool>("logger/update_on_start", false);
  std::string logFileType = param<std::string>("logger/logfile_type", "binary");
  if(logFileType == "csv"){
      managerLoggerOptions.fileTypes = {signal_logger::LogFileType::CSV};
  } else if(logFileType == "bag") {
      managerLoggerOptions.fileTypes = {signal_logger::LogFileType::BAG};
  } else {
      managerLoggerOptions.fileTypes = {signal_logger::LogFileType::BINARY};
  }

  //--- Configure controllers
  {
    boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);

    // Initialize model of the robot and optionally load parameters from a yaml file.
    try {
      std::string anymalUrdfDescription = param<std::string>("/anymal_description", "");
      bool updateInertialParameters = param<bool>("/inertial_parameter_estimation/update_inertial_parameters", false);

      model_.init(timeStep_, isRealRobot_, anymalUrdfDescription);
      if (updateInertialParameters){
        MELO_INFO("[AnymalHighLevelController::init] Updating inertial parameters from YAML.")
        if (!anymal_model_ros::updateInertialParameters(model_.getState()->getAnymalModelPtr(), getNodeHandle())){
          MELO_WARN_STREAM("[RobotModel::initializeFromUrdf] Inertial parameters provided are not valid.");
        }
      }

      // Set leg configurations.
      const std::string legConfig = param<std::string>("joint_states/leg_config", "xx");
      std::unique_ptr<anymal_model::LegConfigurations> legConfigAnymal;
      if (legConfig == "xx") {
        legConfigAnymal.reset(new anymal_model::LegConfigurationXX());
      } else if (legConfig == "oo") {
        legConfigAnymal.reset(new anymal_model::LegConfigurationOO());
      } else if (legConfig == "xo") {
        legConfigAnymal.reset(new anymal_model::LegConfigurationXO());
      } else if (legConfig == "ox") {
        legConfigAnymal.reset(new anymal_model::LegConfigurationOX());
      } else {
        MELO_WARN_STREAM("[AnymalGazeboPlugin::init] Unknown leg configuration " << legConfig << ". Use xx instead.");
        legConfigAnymal.reset(new anymal_model::LegConfigurationXX());
      }
      model_.switchLegConfigurationsTo(*legConfigAnymal.get());
      MELO_INFO_STREAM("[AnymalHighLevelController::init] Use " << legConfig << "-leg configuration.");
    }
    catch (std::exception& ex) {
      MELO_FATAL_STREAM("[High-level controller] Could not initialize the model: " << ex.what());
      exit(-1);
      return false;
    }
    model_.addVariablesToLog();
    signal_logger::add(iterDurationMs_, std::string{"HLControllerIterDuration"});
    signal_logger::add(updateDurationMs_, std::string{"HLControllerUpdateDuration"});
    signal_logger::add(imuMissCounter_, std::string{"HLImuMissCounter"});
    signal_logger::add(actuatorReadMissCounter_, std::string{"HLActuatorMissCounter"});
    signal_logger::add(anymalStateMissCounter_, std::string{"HLAnymalStateMissCounter"});
    signal_logger::add(actuatorCommandsPublisherMissCounter_, std::string{"HLActuatorCommandsPublisherMissCounter"});

    signal_logger::logger->updateLogger();

    // Initialize controller manager.
    rocoma_ros::ControllerManagerRosOptions managerOptions;
    managerOptions.timeStep = timeStep_;
    managerOptions.isRealRobot = isRealRobot_;
    managerOptions.nodeHandle = this->getNodeHandle();
    managerOptions.loggerOptions = managerLoggerOptions;
    managerOptions.emergencyStopMustBeCleared = true;

    controllerManager_.init(managerOptions);
    controllerManager_.setupControllersFromParameterServer( model_.getState(),
                                                            model_.getCommand(),
                                                            model_.getStateMutex(),
                                                            model_.getCommandMutex() );
  }
  //---

  initializeMessages();
  initializeServices();
  initializePublishers();
  initializeSubscribers();

  std::string syncName = param<std::string>("sync_name", "anymal_controller_sync");
  MELO_INFO_STREAM("Sync signal name: " << syncName);
  syncSlave_.reset(new cosmo::SyncSlave(syncName));
  syncSlave_->start();

  ROS_INFO("Start worker...");

    // Add worker to publish ROS messages.
  any_worker::WorkerOptions options;
  options.callback_ = std::bind(&AnymalHighLevelController::updateController, this, std::placeholders::_1);
  options.defaultPriority_ = 10; // this has high priority
  options.name_ = "AnymalHighLevelController::updateController";
  stopUpdating_ = false;
  options.timeStep_ = std::numeric_limits<double>::infinity();
  this->addWorker(options);
  MELO_INFO_STREAM("Anymal Highlevel controller update Controller added");

  any_worker::WorkerOptions pubOptions;
  pubOptions.callback_ = std::bind(&AnymalHighLevelController::publishWorker, this, std::placeholders::_1);
  pubOptions.defaultPriority_ = 0; // this has low priority
  pubOptions.name_ = "AnymalHighLevelController::publishWorker";
  pubOptions.timeStep_ = std::numeric_limits<double>::infinity();
  this->addWorker(pubOptions);
  MELO_INFO_STREAM("Anymal Highlevel publish worker added");

  // Notify the system that the controller has started.
  notificationPublisher_->notify(notification::Level::LEVEL_INFO,
                             std::string{"Starting up."},
                             std::string{"High-level controller is ready."},
                             NOTIFICATION_ID);
  return true;
}


void AnymalHighLevelController::preCleanup()
{
  stopUpdating_ = true;
  cvUpdate_.notify_all();
  syncSlave_->stop(true);
}

bool AnymalHighLevelController::fileHasContent(const std::string& filename) {
  std::ifstream file;
  file.open(filename, std::ios::binary);
  file.seekg(0, std::ios::end);
  file.close();

  return file.tellg() > 0;
}

void AnymalHighLevelController::cleanup() {
  notificationPublisher_->notify(notification::Level::LEVEL_ERROR,
                             std::string{"LMC is shutting down."},
                             std::string{"high-level controller is shutting down."},
                             NOTIFICATION_ID);
  notificationPublisher_->publish();
  controllerManager_.cleanup();
  signal_logger::logger->cleanup();
  parameter_handler::handler->cleanup();
}




void AnymalHighLevelController::initializeMessages() {
  anymal_model_ros::initialize(rosActuatorCommands_);

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
    joystickReadings_.reset(new sensor_msgs::Joy);
    joystickReadings_->header.stamp = ros::Time::now();
    joystickReadings_->axes.resize(7, 0.0);
    joystickReadings_->buttons.resize(15, 0.0);
  }
}

void AnymalHighLevelController::initializeServices() {
  resetStateEstimatorClient_ = serviceClient<anymal_msgs::ResetStateEstimator>("reset_state_estimator", "/reset_state_estimator");
}


void AnymalHighLevelController::initializePublishers() {

  notificationPublisher_.reset(new notification::NotificationPublisher("default", getNodeHandle(), false));

  auto acOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_commands", getNodeHandle());
  acOptions->rosQueueSize_ = 10u;
  acOptions->rosLatch_ = false;
  acOptions->autoPublishRos_ = false;
  actuatorCommandsPublisher_ = cosmo_ros::advertiseShmRos<ActuatorCommandsShm,
                                                    ActuatorCommandsRos,
                                                    anymal_model_ros::conversion_traits::ConversionTraits>("actuator_commands", acOptions);

}

void AnymalHighLevelController::initializeSubscribers() {
  joystickSubscriber_ = subscribe("joy", "/joy", 10, &AnymalHighLevelController::joystickCallback, this, ros::TransportHints().tcpNoDelay());

  auto velocityOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<TwistShm>>("command_velocity", std::bind(&AnymalHighLevelController::velocityCommandsCallback, this, std::placeholders::_1), getNodeHandle());
  velocityOptions->autoSubscribe_ = false;
  velocityOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  velocityOptions->tryRosResubscribing_ = false;

  velocityCommandsSubscriber_ = cosmo_ros::subscribeShmRos<TwistShm, TwistRos, any_measurements_ros::ConversionTraits>("command_velocity", velocityOptions);

  auto poseOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<PoseShm>>("command_pose", std::bind(&AnymalHighLevelController::poseCommandsCallback, this, std::placeholders::_1), getNodeHandle());
  poseOptions->autoSubscribe_ = false;
  poseOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  poseOptions->tryRosResubscribing_ = false;

  poseCommandsSubscriber_ = cosmo_ros::subscribeShmRos<PoseShm, PoseRos, any_measurements_ros::ConversionTraits>("command_pose", poseOptions);

  softEmergencyStopSubscriber_ = subscribe("soft_emcy_stop", "/soft_emcy_stop", 10, &AnymalHighLevelController::softEmergencyStopCallback, this, ros::TransportHints().tcpNoDelay());
  hardEmergencyStopSubscriber_ = subscribe("hard_emcy_stop", "/hard_emcy_stop", 10, &AnymalHighLevelController::hardEmergencyStopCallback, this, ros::TransportHints().tcpNoDelay());

  localizationSubscriber_ = subscribe("localization", "/localization", 10, &AnymalHighLevelController::localizationCallback, this, ros::TransportHints().tcpNoDelay());

  auto imuOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<ImuShm>>("imu", std::bind(&AnymalHighLevelController::imuCallback, this, std::placeholders::_1), getNodeHandle());
  imuOptions->autoSubscribe_ = false;
  imuOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  imuOptions->tryRosResubscribing_ = false;

  imuSubscriber_ = cosmo_ros::subscribeShmRos<ImuShm, ImuRos, any_measurements_ros::ConversionTraits>("imu",imuOptions);

  auto arOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<ActuatorReadingsShm>>("actuator_readings", std::bind(&AnymalHighLevelController::actuatorReadingsCallback, this, std::placeholders::_1), getNodeHandle());
  arOptions->autoSubscribe_ = false;
  arOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  arOptions->tryRosResubscribing_ = false;

  actuatorReadingsSubscriber_ = cosmo_ros::subscribeShmRos<ActuatorReadingsShm,
                                                     ActuatorReadingsRos,
                                                     anymal_model_ros::conversion_traits::ConversionTraits>("actuator_readings",arOptions);

  auto qsOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<AnymalStateShm>>("anymal_state", std::bind(&AnymalHighLevelController::anymalStateCallback, this, std::placeholders::_1), getNodeHandle());
  qsOptions->autoSubscribe_ = false; //will set to false at some point
  qsOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  qsOptions->tryRosResubscribing_ = false;

  anymalStateSubscriber_ = cosmo_ros::subscribeShmRos<AnymalStateShm,
                                                   AnymalStateRos,
                                                   anymal_model_ros::conversion_traits::ConversionTraits>("anymal_state",qsOptions);

}

void AnymalHighLevelController::publishRos()  {
  actuatorCommandsPublisher_->sendRos();
  notificationPublisher_->publish();
}

bool AnymalHighLevelController::updateController(const any_worker::WorkerEvent& event) {
  ROS_INFO("Started updateController");
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
  const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

  while(!stopUpdating_)
  {
    // Waiting for sync msg from simulation or lowlevel ctrl
    auto msg = syncSlave_->waitForSync();

    if (++updateCounterDecimation_ == updateDecimation_)
    {

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

      receiveMeasurements();
      updateJoystickReadings();
      checkForCommandsTimeOut();
      updateVelocityCommands();
      updatePoseCommands();

      controllerManager_.updateController();
      sendCommands();

      signal_logger::logger->collectLoggerData();

      // Notify the publish worker
      updateCounter_++;
      cvUpdate_.notify_all();

      //-- Measure computation time.
      end = std::chrono::steady_clock::now();
      const int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
      updateDurationMs_ = (double)elapsedTimeNSecs*1e-6;

      if (elapsedTimeNSecs > timeStepNSecs)
        MELO_WARN_THROTTLE(3.0, "Computation of high-level controller is not real-time! Computation took: %lf ms. (Warning is throttled: 3s)", (double)elapsedTimeNSecs*1e-6);

      updateCounterDecimation_ = 0;
    }

  }
  return true;
}

void AnymalHighLevelController::receiveMeasurements() {
  // MELO_INFO("Receiving measurements");

  if(!imuSubscriber_->receive(receiveMaxLockTime_))
    ++imuMissCounter_;

  if(!actuatorReadingsSubscriber_->receive(receiveMaxLockTime_))
    ++actuatorReadMissCounter_;

	if(!anymalStateSubscriber_->receive(receiveMaxLockTime_))
    ++anymalStateMissCounter_;

  poseCommandsSubscriber_->receive();
  velocityCommandsSubscriber_->receive();

}


void AnymalHighLevelController::sendCommands() {

	static ActuatorCommandsShm actuatorCommands;
	{
	  boost::shared_lock<boost::shared_mutex> lockModel(mutexModel_);
    model_.getActuatorCommands(actuatorCommands);
	}

	// Convert commands to ROS messages
	any_measurements::Time stamp = any_measurements_ros::fromRos(ros::Time::now());
  for(const auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    actuatorCommands[actuatorEnum].setStamp(stamp);
    series_elastic_actuator_ros::ConvertRosMessages::writeToMessage(rosActuatorCommands_.commands[actuatorId], actuatorCommands[actuatorEnum]);
  }

  // Send
  if(!actuatorCommandsPublisher_->publish(actuatorCommands, rosActuatorCommands_, sendMaxLockTime_)) {
    ++actuatorCommandsPublisherMissCounter_;
  }
}

bool AnymalHighLevelController::update(const any_worker::WorkerEvent& event) {
  // not used
  MELO_WARN_THROTTLE(1.0, "The AnymalHighLevelController node should not run standalone!");
  return true;
}

void AnymalHighLevelController::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
  *joystickReadings_ = *msg;
}

void AnymalHighLevelController::velocityCommandsCallback(const TwistShm& msg) {
  // Ignore old/garbage messages for safety
  any_measurements::Time age = (any_measurements_ros::fromRos(ros::Time::now())-msg.time_);
  if (age.toSeconds() >= commandsTimeOut_) {
    MELO_WARN("Ignored commanded twist which is %lf seconds old. Twist set to zero.", age.toSeconds());
  } else if (!msg.twist_.getVector().allFinite())  {
    MELO_WARN("Ignored commanded twist, contains NaN or Inf. Twist set to zero.");
  } else { // Msg is OK
    boost::unique_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
    velocityCommandsOutdated_ = false;
    velocityCommands_ = msg;
    return;
  }
  // Set default value if msg ignored
  boost::unique_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
  velocityCommands_.time_ = any_measurements_ros::fromRos(ros::Time::now());
  velocityCommands_.twist_.setZero();
}

void AnymalHighLevelController::poseCommandsCallback(const PoseShm& msg) {
  // Ignore old/garbage messages for safety
  any_measurements::Time age = (any_measurements_ros::fromRos(ros::Time::now()) - msg.time_);
  if (age.toSeconds() >= commandsTimeOut_) {
    MELO_WARN("Ignored commanded pose which is %lf seconds old. Pose set to default.", age.toSeconds());
  } else if (!msg.pose_.getTransformationMatrix().allFinite())  {
    MELO_WARN("Ignored commanded pose, contains Nan or Inf. Pose set to default.");
  } else { // Msg is OK
    boost::unique_lock<boost::shared_mutex> lock(mutexPoseCommands_);
    poseCommandsOutdated_ = false;
    poseCommands_ = msg;
    return;
  }
  // Set default value if msg ignored
  boost::unique_lock<boost::shared_mutex> lock(mutexPoseCommands_);
  poseCommands_.time_ = any_measurements_ros::fromRos(ros::Time::now());
  poseCommands_.pose_.setIdentity();
}

void AnymalHighLevelController::updateJoystickReadings() {
  /*
   * If the real robot is controlled and the joystick command is too old,
   * an emergency stop is invoked.
   */
  ros::Duration age;
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
    age = (ros::Time::now()-joystickReadings_->header.stamp);
  }

  if (isRealRobot_ && (age >= ros::Duration(ros::DURATION_MAX)) ) {
    MELO_WARN("Joystick message is %lf seconds old! Called emergency stop!", age.toSec());
    controllerManager_.ControllerManager::emergencyStop();
    {
      boost::unique_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
      joystickReadings_->header.stamp = ros::Time::now();
    }
  }
  else {
    //-- update the model
    {
      boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);
      boost::shared_lock<boost::shared_mutex> lock(mutexJoystickReadings_);
      model_.setJoystickCommands(*joystickReadings_);
    }
    //--
  }
}

void AnymalHighLevelController::updateVelocityCommands() {
  boost::shared_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
  boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);
  model_.setCommandVelocity(velocityCommands_);
}

void AnymalHighLevelController::updatePoseCommands() {
  boost::shared_lock<boost::shared_mutex> lock(mutexPoseCommands_);
  boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);
  model_.setCommandPose(poseCommands_);
}

void AnymalHighLevelController::checkForCommandsTimeOut() {
  any_measurements::Time agePoseCommands = (any_measurements_ros::fromRos(ros::Time::now())-poseCommands_.time_);
  if (agePoseCommands.toSeconds() >= commandsTimeOut_ && !poseCommandsOutdated_) {
    boost::unique_lock<boost::shared_mutex> lock(mutexPoseCommands_);
    poseCommands_.time_ = any_measurements_ros::fromRos(ros::Time::now());
    poseCommands_.pose_.setIdentity();
    poseCommandsOutdated_ = true;
    MELO_DEBUG("Pose timed out, since last message was received %lf seconds ago. Pose set to default.", agePoseCommands.toSeconds());
  }
  any_measurements::Time ageVelocityCommands = (any_measurements_ros::fromRos(ros::Time::now())-velocityCommands_.time_);
  if (ageVelocityCommands.toSeconds() >= commandsTimeOut_ && !velocityCommandsOutdated_) {
    boost::unique_lock<boost::shared_mutex> lock(mutexVelocityCommands_);
    velocityCommands_.time_ = any_measurements_ros::fromRos(ros::Time::now());
    velocityCommands_.twist_.setZero();
    velocityCommandsOutdated_ = true;
    MELO_DEBUG("Twist timed out, since last message was received %lf seconds ago. Twist set to zero.", ageVelocityCommands.toSeconds());
  }
}

void AnymalHighLevelController::softEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg) {
  MELO_WARN("AnymalHighLevelController: Received soft emergency stop.");
  controllerManager_.ControllerManager::emergencyStop();

}
void AnymalHighLevelController::hardEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg) {
  MELO_WARN("AnymalHighLevelController: Received hard emergency stop.");
  controllerManager_.ControllerManager::emergencyStop();
}

void AnymalHighLevelController::localizationCallback(const LocalizationMsg::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);
  model_.setLocalization(*msg);
}

bool AnymalHighLevelController::signalLoggerWorker(const any_worker::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 0lu;

  while(!stopUpdating_)
  {
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

    signal_logger::logger->publishData();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
    const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

    if (elapsedTimeNSecs > timeStepNSecs) {
      if(!isRealRobot_) {
        MELO_WARN_THROTTLE(3.0, "Computation of signal logger of High-level Controller is not real-time! Computation time: %lf ms. (Warning is throttled: 3s)", (double)elapsedTimeNSecs*1e-6);
      }
      else {
        MELO_WARN("Computation of signal logger of High-level Controller is not real-time! Computation time: %lf ms", (double)elapsedTimeNSecs*1e-6);
      }
    }
  }

  return true;
}

bool AnymalHighLevelController::publishWorker(const any_worker::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 01u;

  while(!stopUpdating_)
  {
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

    //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();

    // Do the work
    publishRos();


    signal_logger::logger->publishData();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
    const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

    if (elapsedTimeNSecs > timeStepNSecs) {
      if(!isRealRobot_) {
        MELO_WARN_THROTTLE(3.0, "Computation of publish of High-level Controller is not real-time! Computation time: %lf ms. (Warning is throttled: 3s)", (double)elapsedTimeNSecs*1e-6);
      }
      else {
        MELO_WARN("Computation of publish of High-level Controller is not real-time! Computation time: %lf ms", (double)elapsedTimeNSecs*1e-6);
      }
    }

  }

  return true;
}

void AnymalHighLevelController::imuCallback(const ImuShm& msg) {
  boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);
  model_.setImuMeasurements(msg);
}

void AnymalHighLevelController::actuatorReadingsCallback(const ActuatorReadingsShm& msg) {
  boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);
  model_.setActuatorReadings(msg);
}

void AnymalHighLevelController::anymalStateCallback(const AnymalStateShm& msg) {
  {
    anymalStateStamp_ = msg.time_;
    boost::unique_lock<boost::shared_mutex> lockStamp(mutexAnymalState_);
    anymalState_ = msg;
  }

  {
    boost::unique_lock<boost::shared_mutex> lockModel(mutexModel_);
    model_.setAnymalState(anymalState_);
  }
}

} /* namespace locomotion_controller */
