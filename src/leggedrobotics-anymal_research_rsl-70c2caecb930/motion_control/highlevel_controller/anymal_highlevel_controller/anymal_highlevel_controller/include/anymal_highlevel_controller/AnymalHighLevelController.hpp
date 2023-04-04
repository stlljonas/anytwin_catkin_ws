/*!
 * @file    AnymalHighLevelController.hpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */

#pragma once

#include <ros/ros.h>
#include <any_node/Node.hpp>

// ROS messages / services
#include <anymal_msgs/AnymalState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Joy.h>
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <anymal_msgs/ResetStateEstimator.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include "rocoma_ros/ControllerManagerRos.hpp"
#include <notification/NotificationPublisher.hpp>

#include <kindr/Core>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <memory>
#include <mutex>
#include "ModelData.hpp"

#include <signal_handler/SignalHandler.hpp>
#include <signal_logger/signal_logger.hpp>

#include <cosmo_ros/cosmo_ros.hpp>
#include <cosmo/SyncSlave.hpp>
#include <any_measurements_ros/any_measurements_ros.hpp>
#include <param_io/get_param.hpp>
#include <series_elastic_actuator_ros/series_elastic_actuator_ros.hpp>

#include <anymal_model/AnymalModel.hpp>

#include <anymal_model_ros/conversions.hpp>


namespace anymal_highlevel_controller {
//! High-level controller for ANYmal
/*! This class provides a ROS node to run robot controllers that are loaded via rocoma.
 *
 */
class AnymalHighLevelController : public any_node::Node
{
 public:
  using AD = anymal_description::AnymalDescription;

  typedef kindr::RotationQuaternionPD RotationQuaternion;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;
  typedef kindr::LocalAngularVelocityPD LocalAngularVelocity;
  typedef kindr::AngleAxisPD AngleAxis;
  typedef kindr::Position3D Position;
  typedef kindr::Velocity3D LinearVelocity;
  typedef kindr::VectorTypeless3D Vector;
  typedef any_measurements::Imu ImuShm;
  typedef sensor_msgs::Imu ImuRos;
  typedef anymal_model::ExtendedAnymalState AnymalStateShm;
  typedef anymal_msgs::AnymalState AnymalStateRos;
  using ActuatorReadingsShm = anymal_model::ActuatorReadingRobotContainer;
  using ActuatorReadingsRos = series_elastic_actuator_msgs::SeActuatorReadings;
  using ActuatorCommandsShm = anymal_model::ActuatorCommandRobotContainer;
  using ActuatorCommandsRos = series_elastic_actuator_msgs::SeActuatorCommands;
  using PoseShm = any_measurements::Pose;
  using PoseRos = geometry_msgs::PoseStamped;
  using TwistShm = any_measurements::Twist;
  using TwistRos = geometry_msgs::TwistStamped;

  //! The localization contains the map's pose expressed in the world frame.
  using Localization = any_measurements::PoseWithCovariance;
  using LocalizationMsg = geometry_msgs::PoseWithCovarianceStamped;

 public:
  AnymalHighLevelController() = delete;
  AnymalHighLevelController(any_node::Node::NodeHandlePtr nh);
  virtual ~AnymalHighLevelController() = default;

  bool init() override;
  void preCleanup() override;
  void cleanup() override;
  virtual bool update(const any_worker::WorkerEvent& event);

 protected:
  void initializeMessages();
  void initializeServices();
  void initializePublishers();
  void initializeSubscribers();
  void publishRos();

  void imuCallback(const ImuShm& msg);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void velocityCommandsCallback(const TwistShm& msg);
  void poseCommandsCallback(const PoseShm& msg);
  void softEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);
  void hardEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);
  void localizationCallback(const LocalizationMsg::ConstPtr& msg);

  bool updateController(const any_worker::WorkerEvent& event);
  bool signalLoggerWorker(const any_worker::WorkerEvent& event);
  bool publishWorker(const any_worker::WorkerEvent& event);
  bool fileHasContent(const std::string& file);

  void checkForCommandsTimeOut();
  void updateJoystickReadings();
  void updateVelocityCommands();
  void updatePoseCommands();

  void actuatorReadingsCallback(const ActuatorReadingsShm& msg);
  void anymalStateCallback(const AnymalStateShm& msg);
  void receiveMeasurements();
  void sendCommands();

 protected:
  //! If true, the real robot is controlled and not a simulated.
  bool isRealRobot_;

  //! Time step between two control updates in seconds.
  double timeStep_;

  //! Model of the robot
  ModelData model_;

  //! Mutex to protect the model
  boost::shared_mutex mutexModel_;

  //! Manager of the controllers
  rocoma_ros::ControllerManagerRos<anymal_roco::RocoState, anymal_roco::RocoCommand> controllerManager_;

  //! Joystick commands subscriber.
  ros::Subscriber joystickSubscriber_;

  //! High-level velocity commands like heading, lateral and turning velocity
  cosmo_ros::SubscriberRosPtr<TwistShm, TwistRos, any_measurements_ros::ConversionTraits> velocityCommandsSubscriber_;

  //! High-level pose commands to mainly steer the pose of the torso
  cosmo_ros::SubscriberRosPtr<PoseShm, PoseRos, any_measurements_ros::ConversionTraits> poseCommandsSubscriber_;

  //! Soft emergency stop subscriber which can lead to an emergency stop.
  ros::Subscriber softEmergencyStopSubscriber_;

  //! Hard emergency stop subscriber which can lead to an emergency stop.
  ros::Subscriber hardEmergencyStopSubscriber_;

  //! Localization subscriber.
  ros::Subscriber localizationSubscriber_;

  unsigned long actuatorCommandsPublisherMissCounter_ = 0u;
  cosmo_ros::PublisherRosPtr<ActuatorCommandsShm,
                             ActuatorCommandsRos,
                             anymal_model_ros::conversion_traits::ConversionTraits> actuatorCommandsPublisher_;

  unsigned long imuMissCounter_ = 0u;
  cosmo_ros::SubscriberRosPtr<ImuShm, ImuRos,
                              any_measurements_ros::ConversionTraits> imuSubscriber_;

  unsigned long anymalStateMissCounter_ = 0u;
  cosmo_ros::SubscriberRosPtr<AnymalStateShm, AnymalStateRos,
                              anymal_model_ros::conversion_traits::ConversionTraits> anymalStateSubscriber_;

  unsigned long actuatorReadMissCounter_ = 0u;
  cosmo_ros::SubscriberRosPtr<ActuatorReadingsShm, ActuatorReadingsRos, anymal_model_ros::conversion_traits::ConversionTraits> actuatorReadingsSubscriber_;

  std::unique_ptr<cosmo::SyncSlave> syncSlave_;

  //! Notifies the state of the controller
  std::shared_ptr<notification::NotificationPublisher> notificationPublisher_;

  //! Service to reset the state estimator.
  ros::ServiceClient resetStateEstimatorClient_;

  sensor_msgs::JoyPtr joystickReadings_;
  boost::shared_mutex mutexJoystickReadings_;

  TwistShm velocityCommands_;
  boost::shared_mutex mutexVelocityCommands_;
  bool velocityCommandsOutdated_;

  PoseShm poseCommands_;
  boost::shared_mutex mutexPoseCommands_;
  bool poseCommandsOutdated_;

  double commandsTimeOut_;

  boost::condition_variable_any cvUpdate_;
  boost::mutex mutexPublishUpdate_;
  boost::mutex mutexSignalLoggerUpdate_;
  boost::atomic<bool> stopUpdating_;
  std::atomic<unsigned long> updateCounter_;

  any_measurements::Time anymalStateStamp_;
  boost::shared_mutex mutexAnymalStateStamp_;

  AnymalStateShm anymalState_;
  boost::shared_mutex mutexAnymalState_;

  series_elastic_actuator_msgs::SeActuatorReadingsPtr actuatorReadingsSim_;
  boost::shared_mutex mutexActuatorReadingsSim_;

  ActuatorCommandsRos rosActuatorCommands_;

  std::chrono::time_point<std::chrono::steady_clock> timePrevIteration_;
  double iterDurationMs_;
  double updateDurationMs_;

  unsigned int updateCounterDecimation_ = 0u;
  unsigned int updateDecimation_ = 1u;

  const std::chrono::microseconds receiveMaxLockTime_;
  const std::chrono::microseconds sendMaxLockTime_;

};

} /* namespace locomotion_controller */
