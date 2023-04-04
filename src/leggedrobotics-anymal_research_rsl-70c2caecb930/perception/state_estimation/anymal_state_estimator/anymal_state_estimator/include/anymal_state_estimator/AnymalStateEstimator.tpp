/*!
 * @file    AnymalStateEstimator.tpp
 * @author  Christian Gehring, Markus Staeuble
 * @date    Oct, 2014
 */

// state estimator
#include <anymal_state_estimator/AnymalStateEstimator.hpp>

namespace anymal_state_estimator {

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::AnymalStateEstimator(
    NodeBase::NodeHandlePtr nh)
    :
    Base(nh),
    filter_(nh) {}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::readParameters() {
  robotStateThrottleDecimation_ = NodeBase::param<uint32_t>("publishers/robot_state_throttle/decimation", 40);
  robotStateThrottleCounter_ = 0u;
  MELO_INFO("Robot state throttle decimation: %d", robotStateThrottleDecimation_);

  imuThrottleDecimation_ = NodeBase::param<uint32_t>("publishers/imu_throttle/decimation", 40);
  imuThrottleCounter_ = 0u;
  MELO_INFO("Imu throttle decimation: %d", imuThrottleDecimation_);

  sensorTimeout_ = NodeBase::param<double>("sensor_timeout", 0.1);
  contactTimeout_ = NodeBase::param<double>("contact_timeout", 0.5);
  baseFrameId_ = NodeBase::param<std::string>("base_frame_id", "base");
  odomFrameId_ = NodeBase::param<std::string>("odom_frame_id", "odom");
  imuFrameId_ = NodeBase::param<std::string>("imu_frame_id", "imu");

  initializationDuration_ = NodeBase::param<double>("initialization_duration", 2.0);

  publishRosOdometryMsg_ = NodeBase::param<bool>("publish_ros_odometry_message", false);
  odometryThrottleDecimation_ = NodeBase::param<uint32_t>("publishers/odometry/decimation", 20);
  odometryThrottleCounter_ = 0u;
  MELO_INFO("Odometry throttle decimation: %d", odometryThrottleDecimation_);

  publishZeroVelocityUpdatesMsg_ = NodeBase::param<bool>("publish_zero_velocity_updates_message", true);

  noContactZeroVelUpdIterationThreshold_ =
      NodeBase::param<unsigned int>("no_contact_zvu_iteration_threshold", 1);
  contactZeroVelUpdIterationThreshold_ = NodeBase::param<unsigned int>("contact_zvu_iteration_threshold", 1);

  jointStateThrottleDecimation_ = NodeBase::param<uint32_t>("publishers/joint_state_throttle/decimation", 40);
  jointStateThrottleCounter_ = 0u;
  MELO_INFO("Joint state throttle decimation: %d", jointStateThrottleDecimation_);

  poseInOdomThrottleDecimation_ = NodeBase::param<uint32_t>("publishers/pose_in_odom_throttle/decimation", 40);
  poseInOdomThrottleCounter_ = 0u;
  MELO_INFO("Joint state throttle decimation: %d", poseInOdomThrottleDecimation_);

  twistThrottleDecimation_ = NodeBase::param<uint32_t>("publishers/twist_throttle/decimation", 40);
  twistThrottleCounter_ = 0u;
  MELO_INFO("Joint state throttle decimation: %d", twistThrottleDecimation_);

  contactFilterCoefficientInit_ = NodeBase::param<uint32_t>("contact_filter_coefficient_init", 1u);

  readParametersImpl();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::initializeMessages() {
  initializeRobotStateMsgs();
  estPoseInOdomRos_.header.frame_id = odomFrameId_;
  estTwistRos_.header.frame_id = baseFrameId_;
  estOdometryRos_.header.frame_id = odomFrameId_;
  estOdometryRos_.child_frame_id = baseFrameId_;
  imuAngVelBiasRos_.header.frame_id = imuFrameId_;
  imuLinAccBiasRos_.header.frame_id = imuFrameId_;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::initializePublishers() {

  auto robotStateThrottleOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("robot_state_throttle", this->getNodeHandle());
  robotStateThrottleOptions->rosQueueSize_ = 1u;
  robotStateThrottleOptions->rosLatch_ = false;
  robotStateThrottleOptions->autoPublishRos_ = false;
  robotStateThrottlePublisher_ = cosmo_ros::advertiseShmRos<RobotStateShm, RobotStateRos, RobotStateContainer::template ConversionTrait>(
    "robot_state_throttle", robotStateThrottleOptions);

  auto actReadOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_readings", this->getNodeHandle());
  actReadOptions->rosQueueSize_ = 10u;
  actReadOptions->rosLatch_ = false;
  actReadOptions->autoPublishRos_ = false;
  actuatorReadingsPublisher_ =
      cosmo_ros::advertiseShmRos<ActuatorReadingsShm, ActuatorReadingsRos, ActuatorReadingsContainer::template ConversionTrait>(
          "actuator_readings", actReadOptions);

  auto imuOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("imu", this->getNodeHandle());
  imuOptions->rosQueueSize_ = 10u;
  imuOptions->rosLatch_ = false;
  imuOptions->autoPublishRos_ = false;
  imuPublisher_ = cosmo_ros::advertiseShmRos<ImuShm, ImuRos, ImuContainer::template ConversionTrait>(
      "imu", imuOptions);

  auto imuThrottleOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("imu_throttle", this->getNodeHandle());
  imuThrottleOptions->rosQueueSize_ = 1u;
  imuThrottleOptions->rosLatch_ = false;
  imuThrottleOptions->autoPublishRos_ = false;
  imuPublisherThrottle_ = cosmo_ros::advertiseShmRos<ImuShm, ImuRos, ImuContainer::template ConversionTrait>(
      "imu_throttle", imuThrottleOptions);

  auto jointStateOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("joint_state", this->getNodeHandle());
  jointStateOptions->rosQueueSize_ = 10u;
  jointStateOptions->rosLatch_ = false;
  jointStateOptions->autoPublishRos_ = false;
  jointStatePublisher_ = cosmo_ros::advertiseShmRos<JointStateShm, JointStateRos, any_measurements_ros::ConversionTraits>(
      "joint_state", jointStateOptions);

  auto jointStateThrottleOptions =
      std::make_shared<cosmo_ros::PublisherRosOptions>("joint_state_throttle", this->getNodeHandle());
  jointStateThrottleOptions->rosQueueSize_ = 1u;
  jointStateThrottleOptions->rosLatch_ = false;
  jointStateThrottleOptions->autoPublishRos_ = false;
  jointStateThrottlePublisher_ = cosmo_ros::advertiseShmRos<JointStateShm, JointStateRos, any_measurements_ros::ConversionTraits>(
      "joint_state_throttle", jointStateThrottleOptions);

  auto poseInOdomOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("pose_in_odom", this->getNodeHandle());
  poseInOdomOptions->rosQueueSize_ = 10u;
  poseInOdomOptions->rosLatch_ = false;
  poseInOdomOptions->autoPublishRos_ = false;
  estPoseInOdomPublisher_ =
      cosmo_ros::advertiseShmRos<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>(
          "pose_in_odom", poseInOdomOptions);

  auto twistOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("twist", this->getNodeHandle());
  twistOptions->rosQueueSize_ = 10u;
  twistOptions->rosLatch_ = false;
  twistOptions->autoPublishRos_ = false;
  estTwistPublisher_ =
      cosmo_ros::advertiseShmRos<TwistShm, TwistRos, any_measurements_ros::ConversionTraits>("twist", twistOptions);

  auto poseInOdomThrottleOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("pose_in_odom_throttle", this->getNodeHandle());
  poseInOdomThrottleOptions->rosQueueSize_ = 1u;
  poseInOdomThrottleOptions->rosLatch_ = false;
  poseInOdomThrottleOptions->autoPublishRos_ = false;
  poseInOdomPublisherThrottle_ =
      cosmo_ros::advertiseShmRos<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>(
          "pose_in_odom_throttle", poseInOdomThrottleOptions);

  auto twistThrottleOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("twist_throttle", this->getNodeHandle());
  twistThrottleOptions->rosQueueSize_ = 1u;
  twistThrottleOptions->rosLatch_ = false;
  twistThrottleOptions->autoPublishRos_ = false;
  twistPublisherThrottle_ =
      cosmo_ros::advertiseShmRos<TwistShm, TwistRos, any_measurements_ros::ConversionTraits>("twist_throttle", twistThrottleOptions);

  if (publishRosOdometryMsg_) {
    odometryRosPublisher_ = NodeBase::advertise<OdometryRos>("odometry", "odometry", 1);
  }

  if (publishZeroVelocityUpdatesMsg_) {
    zeroVelocityUpdatesNotificationPublisher_ = NodeBase::advertise<any_msgs::BoolStamped>("zero_velocity_update_notification", "zero_velocity_update_status", 1);
  }

  resetNotificationPublisher_ = NodeBase::advertise<std_msgs::Empty>("reset_notification", "reset_notification", 1);

  imuAngVelRosPublisher_ = NodeBase::advertise<Vec3Ros>("imu_angular_velocity_bias", "imu_angular_velocity_bias", 1);
  imuLinAccRosPublisher_ = NodeBase::advertise<Vec3Ros>("imu_linear_acceleration_bias", "imu_linear_acceleration_bias", 1);

  auto fcOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("force_calibrator_commands", this->getNodeHandle());
  fcOptions->rosQueueSize_ = 10u;
  fcOptions->rosLatch_ = false;
  fcOptions->autoPublishRos_ = false;
  forceCalibratorCommandsPublisher_ =
      cosmo_ros::advertiseShmRos<ForceCalibratorCommandsShm, ForceCalibratorCommandsRos,ForceCalibratorCommandConversionTrait>(
          "force_calibrator_commands", fcOptions);

  notificationPublisher_.reset(new notification::NotificationPublisher("default", this->getNodeHandle(), false));

  std::string contactName;
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    std::string footNameLowerCase = contactKey.getName();
    std::transform(footNameLowerCase.begin(), footNameLowerCase.end(), footNameLowerCase.begin(), ::tolower);
    contactName = "contact_force_" + footNameLowerCase;
    contactWrenchPublishers_[contactEnum] =
        std::unique_ptr<ContactWrenchPublisher>(new ContactWrenchPublisher(contactName, this->getNodeHandle()));
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::initializeSubscribers() {
  auto fccOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<ForceCalibratorCommandsShm>>(
      "force_calibrator_commands",
      std::bind(&AnymalStateEstimator<ConcreteDescription_, RobotState_,
                                      RobotContainersRos_, Filter_>::forceCalibratorCommandsCallback,
                this, std::placeholders::_1),
      this->getNodeHandle());
  fccOptions->autoSubscribe_ = false;
  fccOptions->tryRosResubscribing_ = false;
  fccOptions->rosQueueSize_ = 10u;

  forceCalibratorCommandSubscriber_ =
      cosmo_ros::subscribeShmRos<ForceCalibratorCommandsShm, ForceCalibratorCommandsRos,
                                 ForceCalibratorCommandConversionTrait>("force_calibrator_commands", fccOptions);
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::addVariablesToLog() {
  signal_logger::add(imuMissCount_, std::string{"EstimatorImuMissCount"});
  signal_logger::add(actuatorReadingMissCount_, std::string{"EstimatorActuatorReadingsMisscount"});
  signal_logger::add(imuPublisherMissCount_, std::string{"EstimatorImuPublisherMissCount"});
  signal_logger::add(robotStatePublisherMissCount_, std::string{"EstimatorAnymalStatePublisherMissCount"});
  signal_logger::add(actuatorReadingsPublisherMissCount_, std::string{"EstimatorActuatorReadingsPublisherMissCount"});
  signal_logger::add(robotStateThrottleMissCount_, std::string{"EstimatorAnymalStateThrottleMissCount"});
  std::lock_guard<std::mutex> lock(mutexFilter_);
  filter_.addVariablesToLog();
  addVariablesToLogImpl();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
bool AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::initImpl() {
  const kindr::HomTransformQuatD initialPose(kindr::Position3D(0.0, 0.0, 0.0), kindr::RotationQuaternionD());

  initModel();

  initializeForceCalibrators();
  initializeContactDetectors();
  initializeContactWrenchReaders();

  bool successfulInit = true;
  {
    std::lock_guard<std::mutex> lock(mutexFilter_);
    filter_.setModelPtr(robotModelPtr_);
    successfulInit = filter_.initFilter(initialPose) && successfulInit;
  }

  resetModelState(initialPose);

  notify(notification::Level::LEVEL_INFO, "SE init", "State estimator is initializing.");

  return successfulInit;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::advertiseServices() {
  toggleZeroVelocityUpdatesService_ = NodeBase::advertiseService(
      "toggle_zero_velocity_updates", "toggle_zero_velocity_updates",
      &AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::toggleZeroVelocityUpdatesService,
      this);

  calibrateContactForceService_ = NodeBase::advertiseService(
      "calibrate_forces", "calibrate_forces",
      &AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::calibrateContactForcesService,
      this);
  commandForceCalibratorsService_ = NodeBase::advertiseService(
      "command_force_calib", "command_force_calib",
      &AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::commandForceCalibratorsService,
      this);
  configureForceCalibratorsService_ = NodeBase::advertiseService(
      "configure_force_calib", "configure_force_calib",
      &AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::configureForceCalibratorsService,
      this);
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::receiveMeasurements() {
  if (!forceCalibratorCommandSubscriber_->receive(this->receiveMaxLockTime_)) {
    ++forceCalibratorMissCount_;
  }

  if (!this->imuSubscriber_->receive(this->receiveMaxLockTime_)) {
    ++this->imuMissCount_;
  }

  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    contactWrenchReaders_[contactEnum]->update();
  }

  if (!this->actuatorReadingsSubscriber_->receive(this->receiveMaxLockTime_)) {
    ++actuatorReadingMissCount_;
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::advanceEstimator() {
  const auto stampNow = any_measurements_ros::fromRos(ros::Time::now());
  if (waitingForInitializationDuration_) {
    waitingForInitializationDuration_ = ((stampNow.toSeconds() - fullContactTime_) < initializationDuration_);
  }
  std::lock_guard<std::mutex> lock(mutexFilter_);
  filter_.processKinematics(measJointStates_, contacts_, fakeKinematicsUpdateActive_);
  filter_.processImuReadings(this->imu_);
  filter_.updateFilter();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::preprocessMeasurements() {
  updateContactForces();
  updateContacts();
  fillJointStates();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::updateContactForces() {
  std::lock_guard<std::mutex> lockForceCalibratorCommands(mutexForceCalibratorCommands_);
  std::lock_guard<std::mutex> lockForceCalibrators(mutexForceCalibrators_);

  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactId = contactKey.getId();
    const auto contactEnum = contactKey.getEnum();
    const auto branchEnum = RD::template mapKeyEnumToKeyEnum<ContactEnum, BranchEnum>(contactEnum);

    any_measurements::Wrench wrench;
    wrench = contactWrenchReaders_[contactEnum]->getWrench();

    // Calibrate force and torque.
    forceCalibrators_[contactEnum]->setWrench(wrench.wrench_);
    forceCalibrators_[contactEnum]->command(forceCalibratorCommands_[contactEnum]);
    forceCalibrators_[contactEnum]->advance(this->timeStep_);
    any_measurements::Wrench calibratedWrench;
    calibratedWrench.time_ = wrench.time_;
    forceCalibrators_[contactEnum]->getCalibratedWrench(calibratedWrench.wrench_, wrench.wrench_);
    contactWrenchPublishers_[contactEnum]->setWrench(calibratedWrench);
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::advanceContactDetector(
    ContactEnumContainer<ContactStateEnumDetector>& contactStates){

  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactId = contactKey.getId();
    const auto contactEnum = contactKey.getEnum();
    const auto branchEnum = RD::template mapKeyEnumToKeyEnum<ContactEnum, BranchEnum>(contactEnum);

    auto& contactDetector = contactDetectors_[contactEnum];

    contactDetector->setWrench(contactWrenchPublishers_[contactEnum]->getWrench().wrench_);
    contactDetector->advance(this->timeStep_);

    contactStates[contactEnum] = contactDetector->getContactState();
  }

}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::updateContacts() {

  ContactEnumContainer<ContactStateEnumDetector> contactStates;
  advanceContactDetector(contactStates);

  const auto jointStateMeasTime = measJointStates_.front().time_;
  bool fullContact = true;
  bool noContact = true;

  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactId = contactKey.getId();
    const auto contactEnum = contactKey.getEnum();
    const auto branchEnum = RD::template mapKeyEnumToKeyEnum<ContactEnum, BranchEnum>(contactEnum);

    auto contactFlag = (contactStates[contactEnum] == ContactStateEnumDetector::CLOSED);
    contacts_[contactEnum].state_ = contactStates[contactEnum];

    // reset counter if contact has changed otherwise increment
    if(contactFlag != contacts_[contactEnum].previousFlag_) {
      consecutiveContactCount_[contactEnum] = 0u;
    }
    else if(consecutiveContactCount_[contactEnum] < contactFilterCoefficient_) {
      ++consecutiveContactCount_[contactEnum];
    }

    // update contact flag after "contactFilterCoefficient_" consecutive contact counts
    if(consecutiveContactCount_[contactEnum] >= contactFilterCoefficient_) {
      contacts_[contactEnum].flag_ = contactFlag;
    }

    contacts_[contactEnum].previousFlag_ = contactFlag;

    if (useKfeContactEstimation_ || useContactForceEstimation_) {
      contacts_[contactEnum].stamp_ = jointStateMeasTime;
    } else {
      // TODO: set stamp from readings (reading stamps are not valid at the moment)
      contacts_[contactEnum].stamp_ = jointStateMeasTime;
    }

    contacts_[contactEnum].contactPointInOdom_ = filterState_.getPositionWorldToBaseInWorldFrame() +
        filterState_.getOrientationBaseToWorld().rotate(kindr::Position3D(robotModelPtr_->getPositionBodyToBody(
            BodyEnum::BASE,
            RD::template mapKeyEnumToKeyEnum<ContactEnum, BodyEnum>(contactEnum),
            RD::CoordinateFrameEnum::BASE)));

    if (contacts_[contactEnum].flag_) {
      noContact = false;
      contacts_[contactEnum].lastContactTime_ = contacts_[contactEnum].stamp_;
      contacts_[contactEnum].lastContactPointInOdom_ = contacts_[contactEnum].contactPointInOdom_;
    }

    robotModelPtr_->getContactContainer()[contactEnum]->setState(static_cast<ContactStateEnumDescription>(this->contacts_[contactEnum].state_));

    fullContact &= contacts_[contactEnum].flag_;
  }

  if (waitingForFullContact_ && fullContact) {
    MELO_INFO_STREAM("[AnymalStateEstimator] Estimator has detected full contact, will start initializing.");
    waitingForFullContact_ = false;
    contactFilterCoefficient_ = contactFilterCoefficientUpdate_;
    std::lock_guard<std::mutex> lock(mutexFilter_);
    filter_.updateFullContactFlag(true);
    fullContactTime_ = contacts_[static_cast<typename RD::ContactEnum>(0u)].stamp_.toSeconds();
  }

  if (zeroVelocityUpdatesEnabled_ && noContact) {
    if (!fakeKinematicsUpdateActive_ &&
        (noContactZeroVelUpdIterationCounter_ >= noContactZeroVelUpdIterationThreshold_)) {
      fakeKinematicsUpdateActive_ = true;
      MELO_INFO_STREAM("[AnymalStateEstimator] Activated fake landmarks.");
    } else if (!fakeKinematicsUpdateActive_) {
      noContactZeroVelUpdIterationCounter_++;
      contactZeroVelUpdIterationCounter_ = 0;
    }
  }
  // deactivate if number of consecutive iterations with full contact exceeds contactZeroVelUpdIterationThreshold_
  else {
    noContactZeroVelUpdIterationCounter_ = 0;
    if (fakeKinematicsUpdateActive_ && fullContact) {
      if (contactZeroVelUpdIterationCounter_ >= contactZeroVelUpdIterationThreshold_) {
        fakeKinematicsUpdateActive_ = false;
        MELO_INFO_STREAM("[AnymalStateEstimator] Deactivated fake landmarks.");
      } else {
        contactZeroVelUpdIterationCounter_++;
      }
    }
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::setEstimatedPoses() {
  {
    std::lock_guard<std::mutex> lock(mutexFilter_);
    const auto stamp = filter_.getLastEstimatedStateStamp();

    estPoseInOdom_.time_ = stamp;
    estPoseInOdom_.covariance_ = filter_.getEstPoseInOdomCovariance();

    estTwist_.time_ = stamp;
    estTwist_.covariance_ = filter_.getEstTwistInBaseCovariance();
  }

  // Get pose in odom and twist in base
  const auto& orientationBaseToWorld = filterState_.getOrientationBaseToWorld();

  estPoseInOdom_.pose_.getPosition() = filterState_.getPositionWorldToBaseInWorldFrame();
  estPoseInOdom_.pose_.getRotation() = orientationBaseToWorld;

  estTwist_.twist_.getTranslationalVelocity() = orientationBaseToWorld.inverseRotate(filterState_.getLinearVelocityBaseInWorldFrame());
  estTwist_.twist_.getRotationalVelocity() = filterState_.getAngularVelocityBaseInBaseFrame();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::updateEstimatorStatus() {
  StateStatus prevStatus;
  prevStatus = estimatorStatus_;

  if (waitingForInitializationDuration_) {
    estimatorStatus_ = StateStatus::STATUS_ERROR_UNKNOWN;
  } else {
    std::lock_guard<std::mutex> lock(mutexFilter_);
    estimatorStatus_ = filter_.getStatus();
  }

  const auto stampNow = any_measurements_ros::fromRos(ros::Time::now());

  // if filter status is valid, check sensors
  if (estimatorStatus_ == StateStatus::STATUS_OK) {
    for (const auto contactKey : RD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      const bool hasTimeout = contactWrenchReaders_[contactEnum]->hasTimeout(stampNow.toSeconds());
      if (hasTimeout) {
        hasSensorError_ = true;
        MELO_WARN_THROTTLE(1., "[AnymalStateEstimator] Wrench reader timeout!");
        break;
      }
    }

    bool hasContactTimeout = true;
    if (fakeKinematicsUpdateActive_) {
      hasContactTimeout = false;
    } else {
      for (const auto contactKey : RD::getContactKeys()) {
        const auto contactEnum = contactKey.getEnum();
        const bool hasTimeout =
            stampNow.toSeconds() - contacts_[contactEnum].lastContactTime_.toSeconds() >
            contactTimeout_;
        hasContactTimeout &= hasTimeout;
      }
      if (hasContactTimeout) {
        hasSensorError_ = true;
        MELO_WARN_THROTTLE(1., "[AnymalStateEstimator] Contact timeout!");
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutexFilter_);
      const bool hasImuTimeout =
          stampNow.toSeconds() - filter_.getLastImuStamp().toSeconds() > sensorTimeout_;
      if (hasImuTimeout) {
        hasSensorError_ = true;
        MELO_WARN_THROTTLE(1., "[AnymalStateEstimator] IMU timeout!");
      }
    }
    const bool hasActuatorsTimeout =
        stampNow.toSeconds() - measJointStates_.front().time_.toSeconds() > sensorTimeout_;
    if (hasActuatorsTimeout) {
      hasSensorError_ = true;
      MELO_WARN_THROTTLE(1., "[AnymalStateEstimator] Actuator readings timeout!");
    }

    if (hasSensorError_) {
      estimatorStatus_ = StateStatus::STATUS_ERROR_SENSOR;
    }
  }

  if ((prevStatus != StateStatus::STATUS_OK) &&
      (estimatorStatus_ == StateStatus::STATUS_OK)) {
    notify(notification::Level::LEVEL_INFO, "SE ready", "State estimator is ready!");
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::setOutput() {
  updateEstimatorStatus();
  updateEstimatedRobotState();
  updateEstimatedState();
  setEstimatedPoses();
  // The header frame_id do not get overwritten by these methods
  RobotStateContainer::template ConversionTrait<RobotStateShm, RobotStateRos>::convert(this->estimatedState_,
                                                                                       rosEstRobotState_);
  any_measurements_ros::ConversionTraits<JointStateShm, JointStateRos>::convert(measJointStates_,
                                                                                  measJointStatesRos_);
  any_measurements_ros::ConversionTraits<PoseWithCovarianceShm, PoseWithCovarianceRos>::convert(estPoseInOdom_,
                                                                                                estPoseInOdomRos_);
  any_measurements_ros::ConversionTraits<TwistShm, TwistRos>::convert(estTwist_, estTwistRos_);

  std::lock_guard<std::mutex> lock(mutexFilter_);
  imuAngularVelocityBias_ = filter_.getImuAngularVelocityBias();
  imuLinearAccelerationBias_ = filter_.getImuLinearAccelerationBias();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::updateEstimatedRobotState() {
  std::lock_guard<std::mutex> lock(mutexFilter_);
  filterState_.setPositionWorldToBaseInWorldFrame(filter_.getPositionWorldToBaseInWorldFrame());
  filterState_.setOrientationBaseToWorld(filter_.getOrientationBaseToWorld());
  filterState_.setLinearVelocityBaseInWorldFrame(filter_.getLinearVelocityBaseInWorldFrame());
  filterState_.setAngularVelocityBaseInBaseFrame(filter_.getAngularVelocityBaseInBaseFrame());

  for (auto jointKey : RD::getJointKeys()) {
    const auto jointEnum = jointKey.getEnum();
    const auto jointId = jointKey.getId();
    filterState_.getJointPositions()(jointId) = measJointStates_[jointEnum].position_;
    filterState_.getJointVelocities()(jointId) = measJointStates_[jointEnum].velocity_;
    filterState_.getJointAccelerations()(jointId) = measJointStates_[jointEnum].acceleration_;
    filterState_.getJointTorques()(jointId) = measJointStates_[jointEnum].effort_;
  }

  robotModelPtr_->setState(filterState_, true, true, false);
  framesGeneratorPtr_->update(*robotModelPtr_);

  filterState_.setFrameTransform(CT::FrameTransformEnum::FootprintToOdom, framesGeneratorPtr_->getPoseFootprintToOdom());
  filterState_.setFrameTransform(CT::FrameTransformEnum::FeetcenterToOdom, framesGeneratorPtr_->getPoseFeetcenterToOdom());
  updateAdditionalFrameTransforms();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::publish() {
  {
    std::lock_guard<std::mutex> lock(mutexForceCalibratorCommands_);
    if (!forceCalibratorCommandsPublisher_->publish(forceCalibratorCommands_, this->sendMaxLockTime_)) {
      ++forceCalibratorPublisherMissCount_;
    }
  }

  if (!this->imuPublisher_->publish(this->imu_, this->sendMaxLockTime_)) {
    ++imuPublisherMissCount_;
  }

  if (!imuPublisherThrottle_->publish(this->imu_, this->sendMaxLockTime_)) {
    ++imuThrottleMissCount_;
  }

  for (auto& wrenchManager : contactWrenchPublishers_) {
    if (!wrenchManager->publish(this->sendMaxLockTime_)) {
      ++contactWrenchPublisherMissCount_;
    }
  }

  if (!actuatorReadingsPublisher_->publish(this->actuatorReadings_, this->sendMaxLockTime_)) {
    ++actuatorReadingsPublisherMissCount_;
  }

  if (!jointStatePublisher_->publish(measJointStates_, measJointStatesRos_, this->sendMaxLockTime_)) {
    ++jointStatePublisherMissCount_;
  }

  if (!jointStateThrottlePublisher_->publish(measJointStates_, measJointStatesRos_, this->sendMaxLockTime_)) {
    ++jointStateThrottlePublisherMissCount_;
  }

  estPoseInOdomPublisher_->publish(estPoseInOdom_, estPoseInOdomRos_, this->sendMaxLockTime_);
  estTwistPublisher_->publish(estTwist_, estTwistRos_, this->sendMaxLockTime_);
  poseInOdomPublisherThrottle_->publish(estPoseInOdom_, estPoseInOdomRos_, this->sendMaxLockTime_);
  twistPublisherThrottle_->publish(estTwist_, estTwistRos_, this->sendMaxLockTime_);

  if (!this->robotStatePublisher_->publish(this->estimatedState_, rosEstRobotState_, this->sendMaxLockTime_)) {
    ++robotStatePublisherMissCount_;
  }

  if (!robotStateThrottlePublisher_->publish(this->estimatedState_, rosEstRobotState_, this->sendMaxLockTime_)) {
    ++robotStateThrottleMissCount_;
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::publishRos() {
  this->robotStatePublisher_->sendRos();

  imuPublisher_->sendRos();
  actuatorReadingsPublisher_->sendRos();
  jointStatePublisher_->sendRos();
  estPoseInOdomPublisher_->sendRos();
  estTwistPublisher_->sendRos();
  forceCalibratorCommandsPublisher_->sendRos();
  for (auto& wrenchManager : contactWrenchPublishers_) {
    wrenchManager->sendRos();
  }

  imuThrottleCounter_++;
  if (imuThrottleCounter_ >= imuThrottleDecimation_) {
    imuPublisherThrottle_->sendRos();
    imuThrottleCounter_ = 0;
  }
  robotStateThrottleCounter_++;
  if (robotStateThrottleCounter_ >= robotStateThrottleDecimation_) {
    robotStateThrottlePublisher_->sendRos();
    robotStateThrottleCounter_ = 0;
  }
  jointStateThrottleCounter_++;
  if (jointStateThrottleCounter_ >= jointStateThrottleDecimation_) {
    jointStateThrottlePublisher_->sendRos();
    jointStateThrottleCounter_ = 0;
  }
  poseInOdomThrottleCounter_++;
  if (poseInOdomThrottleCounter_ >= poseInOdomThrottleDecimation_) {
    poseInOdomPublisherThrottle_->sendRos();
    poseInOdomThrottleCounter_ = 0;
  }
  twistThrottleCounter_++;
  if (twistThrottleCounter_ >= twistThrottleDecimation_) {
    twistPublisherThrottle_->sendRos();
    twistThrottleCounter_ = 0;
  }

  odometryThrottleCounter_++;
  if (publishRosOdometryMsg_ && (odometryThrottleCounter_ >= odometryThrottleDecimation_)) {
    if (estTwistRos_.header.stamp.toSec() == estPoseInOdomRos_.header.stamp.toSec()) {
      estOdometryRos_.header.stamp = estTwistRos_.header.stamp;
      estOdometryRos_.pose = estPoseInOdomRos_.pose;
      estOdometryRos_.twist = estTwistRos_.twist;
      odometryRosPublisher_.publish(estOdometryRos_);
      odometryThrottleCounter_ = 0;
    } else {
      MELO_WARN_STREAM("[AnymalStateEstimator] Stamps of estimated twist and pose do not match, dropping ROS odometry message at sequence ID "
          << estOdometryRos_.header.seq);
    }
  }

  imuAngVelBiasRos_.header.stamp = estPoseInOdomRos_.header.stamp;
  imuAngVelBiasRos_.vector.x = imuAngularVelocityBias_(0);
  imuAngVelBiasRos_.vector.y = imuAngularVelocityBias_(1);
  imuAngVelBiasRos_.vector.z = imuAngularVelocityBias_(2);
  imuAngVelRosPublisher_.publish(imuAngVelBiasRos_);

  imuLinAccBiasRos_.header.stamp = estPoseInOdomRos_.header.stamp;
  imuLinAccBiasRos_.vector.x = imuLinearAccelerationBias_(0);
  imuLinAccBiasRos_.vector.y = imuLinearAccelerationBias_(1);
  imuLinAccBiasRos_.vector.z = imuLinearAccelerationBias_(2);
  imuLinAccRosPublisher_.publish(imuLinAccBiasRos_);

  notificationPublisher_->publish();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::resetModules() {
  framesGeneratorPtr_->reset();
  resetModulesImpl();
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
bool AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::resetEstimator(
    const kindr::HomTransformQuatD& pose) {
  std::lock_guard<std::mutex> lock(mutexFilter_);
  filter_.resetFilter(pose);
  filter_.updateFullContactFlag(false);

  hasSensorError_ = false;
  waitingForFullContact_ = true;
  waitingForInitializationDuration_ = true;
  fullContactTime_ = std::numeric_limits<double>::max();
  zeroVelocityUpdatesEnabled_ = false;
  contactFilterCoefficient_ = contactFilterCoefficientInit_;

  resetModelState(pose);
  resetModules();

  Eigen::Vector3d IrIB = pose.getPosition().toImplementation();
  kindr::RotationQuaternionPD qBI = pose.getRotation().inverted();
  if (qBI.norm() == 0) {
    qBI.setIdentity();
  } else {
    qBI.fix();
  }
  std::stringstream description;
  kindr::EulerAnglesZyxPD eulerBI;
  eulerBI = kindr::EulerAnglesZyxPD(qBI);
  eulerBI.setUnique();
  description << "State estimator has been reset to pose x: " << IrIB.x() << " y: " << IrIB.y() << " z: " << IrIB.z()
              << " yaw: " << eulerBI.yaw() << " pitch: " << eulerBI.pitch() << " roll: " << eulerBI.roll();
  notify(notification::Level::LEVEL_INFO, "SE reset.", description.str());

  // Publish a message for notifying this node has been reset.
  publishResetNotification();

  return true;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
bool AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::resetEstimatorHere() {
  // Check turned of for Release because of practical reasons
  // Quickly lifting the robot bring the state estimator into a faulty state
  // The only option is to reset origin which breaks the localization
  // TODO Solve problem in localization module
  // if (estimatorStatus_ == StateStatus::STATUS_OK) {
	kindr::HomTransformQuatD poseBaseToOdom;
	filterState_.getPoseBaseToWorld(poseBaseToOdom);
	resetEstimator(poseBaseToOdom);
	return true;
  // }
  // MELO_WARN_STREAM("[AnymalStateEstimator] Failed to reset estimator here because of invalid status!");
  // return false;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
bool AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::toggleZeroVelocityUpdatesService(
    any_msgs::Toggle::Request& req, any_msgs::Toggle::Response& res) {
  if (req.enable) {
    if (zeroVelocityUpdatesEnabled_) {
      notify(notification::Level::LEVEL_INFO, "SE: Zero Vel Updates", "Zero velocity updates alrady enabled.");
    } else {
      zeroVelocityUpdatesEnabled_ = true;
      notify(notification::Level::LEVEL_INFO, "SE: Zero Vel Updates", "Enabled zero velocity updates.");
    }
  } else {
    if (!zeroVelocityUpdatesEnabled_) {
      notify(notification::Level::LEVEL_INFO, "SE: Zero Vel Updates", "Zero velocity updates alrady disabled.");
    } else {
      zeroVelocityUpdatesEnabled_ = false;
      notify(notification::Level::LEVEL_INFO, "SE: Zero Vel Updates", "Disabled zero velocity updates.");
    }
  }

  if(publishZeroVelocityUpdatesMsg_) {
    publishZeroVelocityUpdatesNotification();
  }

  res.success = true;
  return true;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::notify(
    notification::Level level, const std::string& name, const std::string& description,
    const std::vector<std::string>& outputDevices) {
  switch (level) {
    case notification::Level::LEVEL_FATAL:
      ROS_FATAL_STREAM(description);
      break;
    case notification::Level::LEVEL_ERROR:
      ROS_ERROR_STREAM(description);
      break;
    default:
      MELO_INFO_STREAM(description);
  }

  if (outputDevices.empty()) {
    notificationPublisher_->notify(level, name, description, "ESTIMATOR", 0);
  } else {
    notificationPublisher_->notify(level, name, description, "ESTIMATOR", 0, outputDevices);
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
bool AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::commandForceCalibratorsService(
    anymal_state_estimator::ForceCalibratorCommand::Request& req,
    anymal_state_estimator::ForceCalibratorCommand::Response& res) {
  MELO_INFO_STREAM("commandForceCalibratorsService()");
  std::lock_guard<std::mutex> lockForceCalibratorCommands(mutexForceCalibratorCommands_);
  std::lock_guard<std::mutex> lockForceCalibrators(mutexForceCalibrators_);
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    auto& command = forceCalibratorCommands_[contactEnum];
    command.numSamples_ = req.num_samples;
    command.numGoodSamples_ = req.num_good_samples;
    command.cmdStart_ = req.start;
    command.cmdContinue_ = req.sample;
    command.cmdCalibrate_ = req.calibrate;
    forceCalibrators_[contactEnum]->command(command);
  }

  return true;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
bool AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::configureForceCalibratorsService(
    anymal_state_estimator::ForceCalibratorConfig::Request& req,
    anymal_state_estimator::ForceCalibratorConfig::Response& res) {
  MELO_INFO_STREAM("configureForceCalibratorsService()");
  std::lock_guard<std::mutex> lock(mutexForceCalibrators_);
  try {
    auto calibrator = dynamic_cast<robot_utils::AverageForceCalibrator*>(forceCalibrators_[static_cast<typename RD::ContactEnum>(0u)].get());
    robot_utils::AverageForceCalibrator::Config config;
    calibrator->getConfig(config);
    config.outlierDetectionMaxMahalanobisDistanceForce_ = req.od_distance;
    config.outlierDetectionVariance_ = kindr::WrenchD(
        kindr::Force3D(req.od_variance_x, req.od_variance_y, req.od_variance_z), kindr::Torque3D(1.0, 1.0, 1.0));

    for (const auto contactKey : RD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      auto calib = dynamic_cast<robot_utils::AverageForceCalibrator*>(forceCalibrators_[contactEnum].get());
      calib->setConfig(config);
    }
    res.success = true;
  } catch (...) {
    ;
  }
  return true;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
bool AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::calibrateContactForcesService(
    any_msgs::SetUInt32::Request& req, any_msgs::SetUInt32::Response& res) {
  MELO_INFO_STREAM("calibrateContactForcesService");
  std::lock_guard<std::mutex> lock(mutexForceCalibrators_);
  std::string postfix;
  if (this->isSimulation_) {
    postfix = std::string{"_sim"};
  }
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    if (useForceCalibrators_[contactEnum]) {
      auto avgCalibrator = dynamic_cast<robot_utils::AverageForceCalibrator*>(forceCalibrators_[contactEnum].get());
      robot_utils::AverageForceCalibrator::Config config;
      avgCalibrator->getConfig(config);
      config.numSamples_ = req.data;
      config.enableOutlierDetectorForce_ = false;
      config.enableOutlierDetectorTorque_ = false;
      config.enableForceClamp_ = false;
      avgCalibrator->setConfig(config);
    }

    forceCalibrators_[contactEnum]->startCalibration(false);
  }

  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto& calibrator = forceCalibrators_[contactEnum];
    calibrator->wait();
    MELO_INFO_STREAM("Calibrated force of sensor " << calibrator->getName());
    std::string filename =
        std::string{"contact_force_calibration_"} + calibrator->getName() + postfix + std::string{".txt"};
    calibrator->store(filename);
  }

  res.success = true;
  res.message = "Calibrated contact forces.";
  return true;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::forceCalibratorCommandsCallback(
    const ForceCalibratorCommandsShm& msg) {
  std::lock_guard<std::mutex> lock(mutexForceCalibratorCommands_);
  forceCalibratorCommands_ = msg;
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::initializeContactDetectors() {
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    std::string footNameLowerCase = contactKey.getName();
    std::transform(footNameLowerCase.begin(), footNameLowerCase.end(), footNameLowerCase.begin(), ::tolower);
    std::string detectorName = "contact_detector_" + footNameLowerCase;

    const auto useKfeContactEstimationForContact = NodeBase::param<bool>(detectorName + "/use_kfe_detection", false);
    useKfeContactEstimation_ = useKfeContactEstimation_ || useKfeContactEstimationForContact;
    if (useKfeContactEstimationForContact) {
      double lowerThresholdKfeContactEstimation = NodeBase::param<double>(detectorName + "/lower_kfe_threshold", -2.5);
      double upperThresholdKfeContactEstimation = NodeBase::param<double>(detectorName + "/upper_kfe_threshold", 2.5);

      MELO_INFO_STREAM("Setting config for kfe contact estimation contact: "
                       << detectorName << "\n"
                       << "useKfeContactEstimation_: " << (useKfeContactEstimationForContact ? "yes" : "no") << "\n"
                       << "lowerThresholdKfeContactEstimation_: " << lowerThresholdKfeContactEstimation << "\n"
                       << "upperThresholdKfeContactEstimation_: " << upperThresholdKfeContactEstimation << "\n");

      contactDetectors_[contactEnum] = std::unique_ptr<ContactDetectorFromKFE<ConcreteDescription_, RobotState_>>(
          new ContactDetectorFromKFE<ConcreteDescription_, RobotState_>(
              detectorName, *robotModelPtr_, lowerThresholdKfeContactEstimation, upperThresholdKfeContactEstimation,
              contactKey.getId()));
    } else {
      contactDetectors_[contactEnum] = std::unique_ptr<ContactDetectorThresholdingRos>(
          new ContactDetectorThresholdingRos(this->getNodeHandle(), detectorName));
    }
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::initializeForceCalibrators() {
  std::lock_guard<std::mutex> lock(mutexForceCalibrators_);
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    std::string footNameLowerCase = contactKey.getName();
    std::transform(footNameLowerCase.begin(), footNameLowerCase.end(), footNameLowerCase.begin(), ::tolower);
    std::string calibratorName = "force_calibrator_" + footNameLowerCase;
    useForceCalibrators_[contactEnum] = NodeBase::param<bool>(calibratorName + "/use_force_calibrator", false);

    // force calibrators
    if (useForceCalibrators_[contactEnum]) {
      robot_utils::AverageForceCalibrator::Config config;
      // std::string prefix = calibratorName;
      NodeBase::getParam<double>(calibratorName + std::string{"/filter/alpha/force"}, config.alphaFilterForce_);
      NodeBase::getParam<double>(calibratorName + std::string{"/filter/alpha/torque"}, config.alphaFilterTorque_);
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/max_mahalanobis_distance/force"},
                                 config.outlierDetectionMaxMahalanobisDistanceForce_);
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/max_mahalanobis_distance/torque"},
                                 config.outlierDetectionMaxMahalanobisDistanceTorque_);
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/lower_threshold/force"},
                                 config.lowerForceThreshold_);
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/upper_threshold/force"},
                                 config.upperForceThreshold_);
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/variance/force/x"},
                                 config.outlierDetectionVariance_.getForce().x());
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/variance/force/y"},
                                 config.outlierDetectionVariance_.getForce().y());
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/variance/force/z"},
                                 config.outlierDetectionVariance_.getForce().z());
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/variance/torque/x"},
                                 config.outlierDetectionVariance_.getTorque().x());
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/variance/torque/y"},
                                 config.outlierDetectionVariance_.getTorque().y());
      NodeBase::getParam<double>(calibratorName + std::string{"/outlier_detector/variance/torque/z"},
                                 config.outlierDetectionVariance_.getTorque().z());

      forceCalibrators_[contactEnum] = std::unique_ptr<robot_utils::AverageForceCalibrator>(
          new robot_utils::AverageForceCalibrator(calibratorName, config));
    } else {
      forceCalibrators_[contactEnum] =
          std::unique_ptr<robot_utils::NoneForceCalibrator>(new robot_utils::NoneForceCalibrator(calibratorName));
    }

    //-- Load force calibrations from files
    std::string postfix;
    if (this->isSimulation_) {
      postfix = std::string{"_sim"};
    }
    forceCalibrators_[contactEnum]->load(std::string{"contact_force_calibration_"} +
                                         forceCalibrators_[contactEnum]->getName() + postfix + std::string{".txt"});
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::initializeContactWrenchReaders() {
  const bool useMeasuredAccelerations = NodeBase::param<bool>("use_measured_accelerations", false);
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    std::string footNameLowerCase = contactKey.getName();
    std::transform(footNameLowerCase.begin(), footNameLowerCase.end(), footNameLowerCase.begin(), ::tolower);
    std::string contactName = "contact_force_" + footNameLowerCase;

    // Contact force estimation.
    const auto useContactForceEstimationForContact = NodeBase::param<bool>(contactName + "/use_estimated_force", false);
    // set global flag for contact force estimation if one of the contact detectors uses it
    useContactForceEstimation_ |= useContactForceEstimationForContact;

    MELO_INFO_STREAM("Setting config for contact force estimation for contact force: "
                     << contactName << "\n"
                     << "useContactForceEstimation_: " << (useContactForceEstimationForContact ? "yes" : "no") << "\n");

    if (useContactForceEstimationForContact) {
      if (!contactForceEstimator_) {
        contactForceEstimator_ = std::make_shared<ContactForceEstimation>(*robotModelPtr_, this->timeStep_);
        contactWrenchReaders_[contactEnum] =
            std::unique_ptr<ContactWrenchInterface>(new ContactWrenchEstimator<ConcreteDescription_, RobotState_>(
                contactName, sensorTimeout_, contactEnum, *robotModelPtr_, contactForceEstimator_,
                useMeasuredAccelerations, true));
      } else {
        contactWrenchReaders_[contactEnum] =
            std::unique_ptr<ContactWrenchInterface>(new ContactWrenchEstimator<ConcreteDescription_, RobotState_>(
                contactName, sensorTimeout_, contactEnum, *robotModelPtr_, contactForceEstimator_,
                useMeasuredAccelerations, false));
      }
    } else {
      const auto bodyEnum = RD::template mapEnums<BodyEnum>(contactEnum);
      contactWrenchReaders_[contactEnum] =
          std::unique_ptr<ContactWrenchInterface>(new ContactWrenchReceiver<ConcreteDescription_, RobotState_>(
              contactName, bodyEnum, sensorTimeout_, *robotModelPtr_, this->getNodeHandle()));
    }

    contactWrenchReaders_[contactEnum]->init();
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::publishZeroVelocityUpdatesNotification() {
  if (zeroVelocityUpdatesNotificationPublisher_.getNumSubscribers() > 0u || zeroVelocityUpdatesNotificationPublisher_.isLatched()) {
    auto msg = boost::make_shared<any_msgs::BoolStamped>();
    msg->header.stamp = ros::Time::now();
    msg->value = static_cast<uint8_t>(zeroVelocityUpdatesEnabled_.load());

    zeroVelocityUpdatesNotificationPublisher_.publish(msg);
  }
}

template <typename ConcreteDescription_,
          typename RobotState_,
          typename RobotContainersRos_,
          typename Filter_>
void AnymalStateEstimator<ConcreteDescription_, RobotState_, RobotContainersRos_, Filter_>::publishResetNotification() {
  if (resetNotificationPublisher_.getNumSubscribers() > 0u || resetNotificationPublisher_.isLatched()) {
    auto resetMsg = boost::make_shared<std_msgs::Empty>();
    resetNotificationPublisher_.publish(resetMsg);
  }
}

}  // namespace anymal_state_estimator
