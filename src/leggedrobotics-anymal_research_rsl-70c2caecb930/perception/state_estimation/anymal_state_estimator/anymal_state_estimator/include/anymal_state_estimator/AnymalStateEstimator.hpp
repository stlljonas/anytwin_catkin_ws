/*!
 * @file    AnymalStateEstimator.hpp
 * @author  Christian Gehring, Markus Staeuble
 * @date    Oct, 2014
 */

#pragma once

// ros
#include <ros/ros.h>

#include <any_state_estimator/AnyStateEstimator.hpp>

// ros messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>

// ros srvs
#include <anymal_state_estimator/ForceCalibratorCommand.h>
#include <anymal_state_estimator/ForceCalibratorConfig.h>
#include <robot_utils_ros/ForceCalibratorCommand.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// any msgs
#include <any_msgs/BoolStamped.h>
#include <any_msgs/SetUInt32.h>
#include <any_msgs/Toggle.h>
#include <any_msgs/ExtendedJointState.h>

// any measurements
#include <any_measurements/PoseWithCovariance.hpp>
#include <any_measurements/Time.hpp>
#include <any_measurements/Wrench.hpp>
#include <any_measurements_ros/any_measurements_ros.hpp>

// notification
#include <notification/NotificationPublisher.hpp>

// eigen
#include <Eigen/Core>

// state estimator
#include <anymal_state_estimator/ContactWrenchPublisher.hpp>
#include <anymal_state_estimator/contact_wrench_sources/ContactWrenchInterface.hpp>
#include <anymal_state_estimator/contact_detector_ros/ContactDetectorThresholdingRos.hpp>
#include <anymal_state_estimator/contact_wrench_sources/ContactWrenchEstimator.hpp>
#include <anymal_state_estimator/contact_wrench_sources/ContactWrenchReceiver.hpp>
#include <anymal_state_estimator/contact_detector/ContactDetectorFromKFE.hpp>
#include <anymal_state_estimator/anymal_filter/AnymalFilter.hpp>
#include <anymal_state_estimator/anymal_state_estimator_utils/Contact.hpp>

// basic contact estimation
#include <basic_contact_estimation/ContactDetectorBase.hpp>

// robot utils
#include <robot_utils_ros/ForceCalibratorCommands.h>
#include <robot_utils/force_calibrators/AverageForceCalibrator.hpp>
#include <robot_utils/force_calibrators/ForceCalibratorBase.hpp>
#include <robot_utils/force_calibrators/NoneForceCalibrator.hpp>

// robot utils ros
#include <robot_utils_ros/force_calibrators/ConvertRosMessages.hpp>

// cosmo
#include <cosmo_ros/cosmo_ros.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// kindr
#include <kindr/Core>

// romo
#include <romo/ContactForceEstimation.hpp>
#include <romo/RobotModel.hpp>

// stl
#include <memory>
#include <string>
#include <chrono>
#include <mutex>

// message logger
#include <message_logger/message_logger.hpp>

// anymal_model
#include <anymal_model/AnymalFramesGeneratorBase.hpp>
#include <anymal_model/StateStatus.hpp>

namespace anymal_state_estimator {

/**
 * @brief      Base class used for state estimation for a generic anymal type
 *             robot with custom attachments
 *
 * @tparam     ConcreteDescription_  romo::RobotDescription
 * @tparam     RobotState_           romo::RobotState
 * @tparam     RobotContainersRos_   romo::RobotContainersRos
 * @tparam     Filter_               The filter type. Needs to derive from AnymalFilter
 */
template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
class AnymalStateEstimator : public any_state_estimator::AnyStateEstimator<RobotContainersRos_> {

static_assert(std::is_base_of<AnymalFilter<ConcreteDescription_, RobotState_>, Filter_>::value,
              "[AnymalStateEstimator]: Filter_ must derive from AnymalFilter");

public:
  using NodeBase = any_node::Node;
  using Base = any_state_estimator::AnyStateEstimator<RobotContainersRos_>;

  using typename Base::ActuatorReadingsContainer;
  using typename Base::ActuatorReadingsShm;
  using typename Base::ActuatorReadingsRos;
  using typename Base::ImuContainer;
  using typename Base::ImuShm;
  using typename Base::ImuRos;
  using typename Base::RobotStateContainer;
  using typename Base::RobotStateRos;
  using typename Base::RobotStateShm;

  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using ContactForceEstimation = romo::ContactForceEstimation<ConcreteDescription_, RobotState_>;

  using RD = typename RobotModel::RD;
  using CT = typename ConcreteDescription_::ConcreteTopology;

  using BranchEnum = typename RD::BranchEnum;
  using BodyEnum = typename RD::BodyEnum;
  using JointEnum = typename RD::JointEnum;
  using ContactEnum = typename RD::ContactEnum;
  using ActuatorEnum = typename RD::ActuatorEnum;

  using ContactStateEnumDetector = typename basic_contact_estimation::ContactDetectorBase::ContactState;
  using ContactStateEnumDescription = typename RD::ContactStateEnum;

  using ForceCalibratorCommandsShm = std_utils::EnumArray<ContactEnum, robot_utils::ForceCalibratorCommand>;
  using ForceCalibratorCommandsRos = robot_utils_ros::ForceCalibratorCommands;
  using PoseWithCovarianceShm = any_measurements::PoseWithCovariance;
  using PoseWithCovarianceRos = geometry_msgs::PoseWithCovarianceStamped;
  using OdometryRos = nav_msgs::Odometry;

  using JointStateShm = std_utils::EnumArray<JointEnum, any_measurements::ExtendedJointState>;
  using JointStateRos = any_msgs::ExtendedJointState;

  using TwistShm = any_measurements::TwistWithCovariance;
  using TwistRos = geometry_msgs::TwistWithCovarianceStamped;
  using AnymalFramesGeneratorBase = anymal_model::AnymalFramesGeneratorBase<ConcreteDescription_, RobotState_>;
  using StateStatus = anymal_model::StateStatus;

  using Vec3Ros = geometry_msgs::Vector3Stamped;

  template <typename ValueType_>
  using ContactEnumContainer = std_utils::EnumArray<ContactEnum, ValueType_>;

  template <typename Msg_, typename MsgRos_>
  using ForceCalibratorCommandConversionTrait = robot_utils_ros::ConversionTraits<RD, Msg_, MsgRos_>;

  AnymalStateEstimator() = delete;
  explicit AnymalStateEstimator(any_node::Node::NodeHandlePtr nh);

  ~AnymalStateEstimator() override = default;

 protected:

  bool initImpl() final;
  bool resetEstimator(const kindr::HomTransformQuatD& pose) final;
  bool resetEstimatorHere() final;
  void readParameters() final;
  void initializeMessages() final;
  void initializePublishers() final;
  void initializeSubscribers() final;
  void advertiseServices() final;
  void receiveMeasurements() final;
  void preprocessMeasurements() final;
  void advanceEstimator() final;
  void setOutput() final;
  void publish() final;
  void publishRos() final;
  void addVariablesToLog() final;

  /**
   * @brief      Resets base pose of the robotModelPtr to pose and the actuators
   *             according to the current measurements
   *
   * @param[in]  pose  The pose to reset
   */
  virtual void resetModelState(const kindr::HomTransformQuatD& pose) = 0;

  /**
   * @brief      Initializes robot specific ros messages
   */
  virtual void initializeRobotStateMsgs() = 0;

  /**
   * @brief      Fills the joint states with new measurements
   */
  virtual void fillJointStates() = 0;


  /**
   * @brief      Updates the robot specific estimated state
   */
  virtual void updateEstimatedState() = 0;

  /**
   * @brief      Sets the model and frames generator ptr
   */
  virtual void initModel() = 0;

  /**
   * @brief      Optionally implements setting additional
   *             frame transforms in updateEstimatedRobotState
   */
  virtual void updateAdditionalFrameTransforms() {  /*do nothing*/  };

  /**
   * @brief      Read paramters for further objects
   */
  virtual void readParametersImpl() { /*do nothing*/ }

  /**
   * @brief      Add logger variables for further objects
   */
  virtual void addVariablesToLogImpl() { /*do nothing*/ }

  /**
   * @brief      Reset further modules/resources
   */
  virtual void resetModulesImpl() { /*do nothing*/ }

  /**
   * @brief      ReInitializes additional resources/modules such as contact detectors or the frames generator
   */
  void resetModules();

  /**
   * @brief      Handles incoming forcecalibrator commands from the highlevel
   *             controller
   *
   * @param[in]  msg   The force calibrator command msg
   */
  void forceCalibratorCommandsCallback(const ForceCalibratorCommandsShm& msg);

  /**
   * @brief      Enables and disables zero velocity updates. If enabled, the
   *             estimator will try to keep the base fixed with respect to the
   *             world when contact is lost completely. Also, contact timeouts
   *             are circumvented in this case.
   *
   * @param      req   The request
   * @param      res   The resource
   *
   * @return     true if successful
   */
  bool toggleZeroVelocityUpdatesService(any_msgs::Toggle::Request& req, any_msgs::Toggle::Response& res);

  /**
   * @brief      Sends notifications over various channels as specified in the
   *             estimator parameters
   *
   * @param[in]  level          The level
   * @param[in]  name           The name
   * @param[in]  description    The description
   * @param[in]  outputDevices  The output devices
   */
  void notify(notification::Level level, const std::string& name, const std::string& description,
              const std::vector<std::string>& outputDevices = std::vector<std::string>{});

  /**
   * @brief      Sets the calibrators in calibration mode
   *
   * @param      req   The request
   * @param      res   The resource
   *
   * @return     true if successful
   */
  bool calibrateContactForcesService(any_msgs::SetUInt32::Request& req, any_msgs::SetUInt32::Response& res);

  /**
   * @brief      Handles incoming force calibrator commands
   *
   * @param      req   The request
   * @param      res   The resource
   *
   * @return     true if successful
   */
  bool commandForceCalibratorsService(anymal_state_estimator::ForceCalibratorCommand::Request& req,
                                      anymal_state_estimator::ForceCalibratorCommand::Response& res);

  /**
   * @brief      Handles incoming force calibrator configurations
   *
   * @param      req   The request
   * @param      res   The resource
   *
   * @return     true if successful
   */
  bool configureForceCalibratorsService(anymal_state_estimator::ForceCalibratorConfig::Request& req,
                                        anymal_state_estimator::ForceCalibratorConfig::Response& res);

  void initializeForceCalibrators();
  virtual void initializeContactDetectors();
  virtual void initializeContactWrenchReaders();

  /**
   * @brief      Processes measured or estimated contact forces using the
   *             calibrators
   */
  void updateContactForces();

  /**
   * @brief      Detects whether the endeffectors are in contact
   *
   * @param      contactStates   Container to store the resuls in
   */
  virtual void advanceContactDetector(ContactEnumContainer<ContactStateEnumDetector>& contactStates);

  /**
   * @brief      Advances the contact detectors and updates related logic
   */
  void updateContacts();

  /**
   * @brief      Updates the estimated anymalState with the newest estimate
   *             from the filter
   */
  void updateEstimatedRobotState();
  void setEstimatedPoses();
  void updateEstimatorStatus();

  /**
   * Publish a message that contains the zero velocity updates status (enable/disabled).
   */
  void publishZeroVelocityUpdatesNotification();

  /**
   * Publish a message notifying that the state estimator has been reset.
   */
  void publishResetNotification();

 protected:
  bool hasSensorError_{false};
  bool useKfeContactEstimation_{false};
  bool useContactForceEstimation_{false};
  bool hasFullContact_{false};
  bool waitingForFullContact_{true};
  bool waitingForInitializationDuration_{true};
  bool fakeKinematicsUpdateActive_{false};
  //flags which can be toggled via service calls and should therefore be threadsafe
  std::atomic<bool> zeroVelocityUpdatesEnabled_{false};
  std::atomic<bool> usePoseMeas_{true};

  bool publishRosOdometryMsg_{false};
  bool publishZeroVelocityUpdatesMsg_{false};

  Eigen::Matrix<double,3,1> imuLinearAccelerationBias_{Eigen::Matrix<double,3,1>::Zero()};
  Eigen::Matrix<double,3,1> imuAngularVelocityBias_{Eigen::Matrix<double,3,1>::Zero()};

  double initializationDuration_{0.0};
  double fullContactTime_{std::numeric_limits<double>::max()};
  double sensorTimeout_{0.0};
  double contactTimeout_{0.0};

  unsigned int noContactZeroVelUpdIterationCounter_{0u};
  unsigned int noContactZeroVelUpdIterationThreshold_{1u};
  unsigned int contactZeroVelUpdIterationCounter_{0u};
  unsigned int contactZeroVelUpdIterationThreshold_{1u};

  std::string baseFrameId_{""};
  std::string odomFrameId_{""};
  std::string imuFrameId_{""};

  //mutexes to lock resources used in timecritical roscallbacks (e.g. poseInMap callbacks, force calibrator callbacks)
  std::mutex mutexFilter_;
  std::mutex mutexForceCalibrators_;
  std::mutex mutexForceCalibratorCommands_;

  Filter_ filter_;
  std::shared_ptr<RobotModel> robotModelPtr_{nullptr};

  std::unique_ptr<AnymalFramesGeneratorBase> framesGeneratorPtr_{nullptr};
  StateStatus estimatorStatus_{StateStatus::STATUS_ERROR_UNKNOWN};
  RobotState_ filterState_;

  std::atomic<unsigned int> contactFilterCoefficient_{1u};
  const unsigned int contactFilterCoefficientUpdate_{1u};
  unsigned int contactFilterCoefficientInit_{200u};
  ContactEnumContainer<unsigned int> consecutiveContactCount_{0u};
  ContactEnumContainer<std::unique_ptr<ContactWrenchInterface>> contactWrenchReaders_;
  ContactEnumContainer<std::unique_ptr<ContactWrenchPublisher>> contactWrenchPublishers_;
  ContactEnumContainer<std::unique_ptr<robot_utils::ForceCalibratorBase>> forceCalibrators_;
  ContactEnumContainer<std::unique_ptr<basic_contact_estimation::ContactDetectorBase>> contactDetectors_;
  ContactEnumContainer<Contact> contacts_;
  ContactEnumContainer<bool> useForceCalibrators_;

  std::shared_ptr<ContactForceEstimation> contactForceEstimator_{nullptr};

  RobotStateRos rosEstRobotState_;

  JointStateShm measJointStates_;
  JointStateRos measJointStatesRos_;

  PoseWithCovarianceShm estPoseInOdom_;
  PoseWithCovarianceRos estPoseInOdomRos_;

  TwistShm estTwist_;
  TwistRos estTwistRos_;

  OdometryRos estOdometryRos_;

  Vec3Ros imuAngVelBiasRos_;
  Vec3Ros imuLinAccBiasRos_;

  ForceCalibratorCommandsShm forceCalibratorCommands_;

  // cosmo ros miss counts
  unsigned int forceCalibratorMissCount_{0u};
  unsigned int contactWrenchPublisherMissCount_{0u};
  unsigned int forceCalibratorPublisherMissCount_{0u};
  unsigned int jointStatePublisherMissCount_{0u};
  unsigned int jointStateThrottlePublisherMissCount_{0u};
  unsigned int robotStateThrottleMissCount_{0u};
  unsigned int imuMissCount_{0u};
  unsigned int actuatorReadingMissCount_{0u};
  unsigned int imuPublisherMissCount_{0u};
  unsigned int imuThrottleMissCount_{0u};
  unsigned int robotStatePublisherMissCount_{0u};
  unsigned int actuatorReadingsPublisherMissCount_{0u};

  // throttled publisher settings
  unsigned int jointStateThrottleCounter_{0u};
  unsigned int poseInOdomThrottleCounter_{0u};
  unsigned int twistThrottleCounter_{0u};
  unsigned int imuThrottleCounter_{0u};
  unsigned int robotStateThrottleCounter_{0u};
  unsigned int odometryThrottleCounter_{0u};

  unsigned int imuThrottleDecimation_{1u};
  unsigned int twistThrottleDecimation_{1u};
  unsigned int poseInOdomThrottleDecimation_{1u};
  unsigned int jointStateThrottleDecimation_{1u};
  unsigned int robotStateThrottleDecimation_ {1u};
  unsigned int odometryThrottleDecimation_ {1u};

  // Subscribers
  cosmo_ros::SubscriberRosPtr<ForceCalibratorCommandsShm, ForceCalibratorCommandsRos,
                              ForceCalibratorCommandConversionTrait>
      forceCalibratorCommandSubscriber_;

  // Publishers
  cosmo_ros::PublisherRosPtr<ForceCalibratorCommandsShm, ForceCalibratorCommandsRos,
                             ForceCalibratorCommandConversionTrait>
      forceCalibratorCommandsPublisher_;
  cosmo_ros::PublisherRosPtr<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>
      estPoseInOdomPublisher_;
  cosmo_ros::PublisherRosPtr<TwistShm, TwistRos, any_measurements_ros::ConversionTraits> estTwistPublisher_;
  cosmo_ros::PublisherRosPtr<JointStateShm, JointStateRos, any_measurements_ros::ConversionTraits> jointStatePublisher_;
  cosmo_ros::PublisherRosPtr<ImuShm, ImuRos, ImuContainer::template ConversionTrait> imuPublisher_;
  cosmo_ros::PublisherRosPtr<ActuatorReadingsShm,
                             ActuatorReadingsRos,
                             ActuatorReadingsContainer::template ConversionTrait> actuatorReadingsPublisher_;

  // Throttle publishers
  cosmo_ros::PublisherRosPtr<JointStateShm, JointStateRos, any_measurements_ros::ConversionTraits> jointStateThrottlePublisher_;
  cosmo_ros::PublisherRosPtr<RobotStateShm, RobotStateRos, RobotStateContainer::template ConversionTrait>
      robotStateThrottlePublisher_;
  cosmo_ros::PublisherRosPtr<ImuShm, ImuRos, ImuContainer::template ConversionTrait> imuPublisherThrottle_;
  cosmo_ros::PublisherRosPtr<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>
      poseInOdomPublisherThrottle_;
  cosmo_ros::PublisherRosPtr<TwistShm, TwistRos, any_measurements_ros::ConversionTraits> twistPublisherThrottle_;

  // Other publishers
  ros::Publisher odometryRosPublisher_;
  ros::Publisher zeroVelocityUpdatesNotificationPublisher_;
  ros::Publisher imuAngVelRosPublisher_;
  ros::Publisher imuLinAccRosPublisher_;
  ros::Publisher resetNotificationPublisher_;
  std::shared_ptr<notification::NotificationPublisher> notificationPublisher_;

  // Services
  ros::ServiceServer toggleZeroVelocityUpdatesService_;
  ros::ServiceServer setForceCalibrationAlhpaFilterService_;
  ros::ServiceServer setForceCalibrationMaxMahalanobisDistanceService_;
  ros::ServiceServer calibrateContactForceService_;
  ros::ServiceServer commandForceCalibratorsService_;
  ros::ServiceServer configureForceCalibratorsService_;

};

}  // namespace anymal_state_estimator

// gets compiled by AnymalStateEstimatorDefault.cpp
// #include <anymal_state_estimator/AnymalStateEstimator.tpp>
