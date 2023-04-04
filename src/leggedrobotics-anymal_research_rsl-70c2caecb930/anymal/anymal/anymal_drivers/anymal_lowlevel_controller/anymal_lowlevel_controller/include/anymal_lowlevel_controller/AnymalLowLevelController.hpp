#pragma once


// std msgs
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

// any msgs
#include <any_msgs/State.h>
#include <any_msgs/Toggle.h>

// any node
#include <any_node/Node.hpp>
#include <any_node/ThreadedPublisher.hpp>

// any measurements ros
#include <any_measurements_ros/any_measurements_ros.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// anymal model
#include <anymal_model/actuator_containers.hpp>

// anymal model ros
#include <anymal_model_ros/conversion_traits/ConversionTraits.hpp>

// anymal lowlevel controller msgs
#include <anymal_msgs/AnymalLowLevelControllerGoToState.h>
#include <anymal_msgs/AnymalLowLevelControllerState.h>

// romo std
#include <romo_std/common/actuator_containers.hpp>

// cosmo
#include <cosmo/SyncMaster.hpp>

// cosmo ros
#include <cosmo_ros/cosmo_ros.hpp>

// series elastic actuator ros
#include <series_elastic_actuator_ros/series_elastic_actuator_ros.hpp>

// anymal lowlevel controller
#include "anymal_lowlevel_controller/state_machine/states.hpp"
#include "anymal_lowlevel_controller/System.hpp"


namespace anymal_lowlevel_controller {


class AnymalLowLevelController : public any_node::Node
{
public:
  using AD = anymal_description::AnymalDescription;

  using ActuatorReadingsShm = anymal_model::ActuatorReadingRobotContainer ;
  using ActuatorReadingsRos = series_elastic_actuator_msgs::SeActuatorReadings;
  using ActuatorReadingsExtended = romo_std::ActuatorReadingContainer<AD, series_elastic_actuator::SeActuatorReadingExtended>;
  using ActuatorCommandsShm = anymal_model::ActuatorCommandRobotContainer ;
  using ActuatorCommandsRos = series_elastic_actuator_msgs::SeActuatorCommands;
  using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;
  using GoToState = anymal_msgs::AnymalLowLevelControllerGoToState;

  AnymalLowLevelController(NodeHandlePtr nh);
  ~AnymalLowLevelController() override = default;

  bool init() override;
  void preCleanup() override;
  void cleanup() override;

  void notifyControllerStatus(const bool running);
  bool toggleActuatorCommunication(any_msgs::Toggle::Request& req, any_msgs::Toggle::Response& res);

protected:
  void updateCommands();
  void updateActiveSubstates();
  void updateMeasurements();

  void softEmergencyStopCb(const std_msgs::Empty::ConstPtr& msg);
  void hardEmergencyStopCb(const std_msgs::Empty::ConstPtr& msg);

  bool goToStateCb(GoToState::Request& req, GoToState::Response& res);
  void actuatorCommandsCallback(const ActuatorCommandsShm& msg);

  bool updateController(const any_worker::WorkerEvent& event);
  bool publishWorker(const any_worker::WorkerEvent& workerEvent);
  bool signalLoggerWorker(const any_worker::WorkerEvent& event);

  void publishDysfunctionState();

protected:
  bool runPublishers_ = true;
  double timeStep_ = 0.0025;

  SystemPtr system_;
  std::atomic<bool> freezesAreSet_; // True if only freezes are set by setActuatorCommands()

  state_machine::StateMachinePtr stateMachine_;

  ActuatorCommandsShm actuatorCommands_;
  ActuatorReadingsShm actuatorReadings_;
  ActuatorReadingsExtended actuatorReadingsExtended_;

  ros::Subscriber softEmergencyStopSubscriber_;
  ros::Subscriber hardEmergencyStopSubscriber_;
  ros::Publisher softEmergencyStopPublisher_;
  ros::ServiceServer goToStateServer_;

  // Member variables accessed in publishing thread.
  std::mutex publishMembersMutex_;
  state_machine::StateEnum lastActiveStateEnum_ = state_machine::StateEnum::NA;
  double batteryVoltage_ = 0.0;
  unsigned int publishBatteryVoltageCounter_ = 0;
  const unsigned int publishBatteryVoltageDecimation_ = 100;
  ros::Publisher activeStatePublisher_;
  ros::Publisher batteryVoltagePublisher_;
  ros::Publisher dysfunctionPublisher_;

  ros::ServiceServer enableActuatorComService_;
  std::atomic<bool> hasDysfunction_;
  std::atomic<bool> hasDysfunctionPublished_;

  unsigned int actuatorCommandsMissCount_ = 0u;
  ros::Time lastActuatorCommandTimestamp_;
  ros::Duration actuatorCommandTimeout_{0.1};
  cosmo_ros::SubscriberRosPtr<ActuatorCommandsShm,
                              ActuatorCommandsRos,
                              anymal_model_ros::conversion_traits::ConversionTraits> actuatorCommandsSubscriber_;

  unsigned int actuatorReadingsMissCount_ = 0u;
  cosmo_ros::PublisherRosPtr<ActuatorReadingsShm,
                             ActuatorReadingsRos,
                             anymal_model_ros::conversion_traits::ConversionTraits> actuatorReadingsPublisher_;

  std::unique_ptr<cosmo::SyncMaster> syncMaster_;

  unsigned int workingCounterTooLowCount_ = 0u;

  std::chrono::time_point<std::chrono::steady_clock> timePrevIteration_;
  double iterDurationMs_;
  double updateDurationMs_;
  // double durationSubinterval1_;
  // double durationSubinterval2_;
  // double durationSubinterval3_;
  // double durationSubinterval4_;
  // double durationSubinterval5_;
  // double durationSubinterval6_;
  // double durationSubinterval7_;
  // double durationSubinterval8_;
  // double durationSubinterval9_;
  // double durationSubinterval10_;

  boost::condition_variable_any cvUpdate_;
  boost::mutex mutexSignalLoggerUpdate_;
  boost::mutex mutexPublishUpdate_;
  boost::atomic<bool> stopUpdating_;
  std::atomic<unsigned long> updateCounter_;

  const std::chrono::microseconds receiveMaxLockTime_;
  const std::chrono::microseconds sendMaxLockTime_;

  signal_logger::LogFileTypeSet logFileTypes_;
};


} // anymal_lowlevel_controller
