// param io
#include <param_io/get_param.hpp>

// anymal msgs
#include <anymal_msgs/Contact.h>

// anymal model
#include <anymal_model/actuator_containers.hpp>

// anymal model ros
#include <anymal_model_ros/conversions.hpp>
#include <anymal_model_ros/initializations.hpp>

// romo std
#include <romo_std/common/container_utils.hpp>

// anydrive ros
#include <anydrive_ros/conversions.hpp>

// series elastic actuator anydrive
#include <series_elastic_actuator_anydrive/conversions.hpp>

// anymal gazebo
#include "anymal_gazebo/AnymalGazeboPlugin.hpp"

namespace gazebo {

AnymalGazeboPlugin::AnymalGazeboPlugin() : SeaGazeboPluginBase("Anymal") {}

void AnymalGazeboPlugin::Reset() {
  // Reset the base class.
  SeaGazeboPluginBase::Reset();

  // Reset the frames generator.
  framesGenerator_.reset();
}

void AnymalGazeboPlugin::readParameters() {
  param_io::getParam(*nodeHandle_, "real_time_update_rate", realTimeUpdateRate_);
  param_io::getParam(*nodeHandle_, "max_step_size", maxStepSize_);
  param_io::getParam(*nodeHandle_, "publishing_time_step", publishingTimeStep_);

  param_io::getParam(*nodeHandle_, "simulate_estimator", simulateEstimator_);

  param_io::getParam(*nodeHandle_, "publish_groundtruth", publishGroundtruth_);
  param_io::getParam(*nodeHandle_, "frame/base/name", frameBase_);
  param_io::getParam(*nodeHandle_, "frame/odometry/name", frameOdometry_);
  param_io::getParam(*nodeHandle_, "frame/world/name", frameWorld_);
  param_io::getParam(*nodeHandle_, "frame/footprint/name", frameFootprint_);
  param_io::getParam(*nodeHandle_, "frame/feetcenter/name", frameFeetcenter_);

  param_io::getParam(*nodeHandle_, "frame/odometry/offset/x", frameOdometryOffsetX_);
  param_io::getParam(*nodeHandle_, "frame/odometry/offset/y", frameOdometryOffsetY_);
  param_io::getParam(*nodeHandle_, "frame/odometry/offset/z", frameOdometryOffsetZ_);

  param_io::getParam(*nodeHandle_, "contact_forces/linear_velocity_threshold", contactLinearVelocityThreshold_);

  std::string legConfig = "xx";
  param_io::getParam(*nodeHandle_, "joint_states/leg_config", legConfig);
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
    MELO_WARN_STREAM("[AnymalGazeboPlugin::readParameters] Unknown leg configuration " << legConfig << ". Use xx instead.");
    legConfigAnymal.reset(new anymal_model::LegConfigurationXX());
  }

  std::vector<double> jointPositionsDefault;
  param_io::getParam(*nodeHandle_, "joint_states/default_positions_" + legConfig, jointPositionsDefault);

  for (const auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    jointPositionsDefault_[actuatorEnum] = jointPositionsDefault[actuatorId];
  }

  // Initialize map from collision link names to publisher names.
  collisionLinkNamesToPublisherNames_[AD::ContactEnum::LF_FOOT] = std::string{"contact_force_lf_foot"};
  collisionLinkNamesToPublisherNames_[AD::ContactEnum::RF_FOOT] = std::string{"contact_force_rf_foot"};
  collisionLinkNamesToPublisherNames_[AD::ContactEnum::LH_FOOT] = std::string{"contact_force_lh_foot"};
  collisionLinkNamesToPublisherNames_[AD::ContactEnum::RH_FOOT] = std::string{"contact_force_rh_foot"};

  // Initialize anymal model.
  const std::string anymalUrdfDescription = param_io::param<std::string>(*nodeHandle_, "/anymal_description", "");
  anymalModel_.initializeFromUrdf(anymalUrdfDescription);
  anymal_model_ros::initialize(anymalMsgRos_);
  anymal_model::initializeActuatorCommandsFromLimits(actuatorDefaultPositionCommands_, *anymalModel_.getLimitsAnymal(), *legConfigAnymal);
  readingsExtendedMsg_.readings.resize(anymal_description::AnymalDescription::getActuatorsDimension());
}

void AnymalGazeboPlugin::initSubscribers() {
  // Actuator commands.
  auto optionsActuatorCommands = std::make_shared<cosmo_ros::SubscriberRosOptions<ActuatorCommandsShm>>(
      "actuator_commands", std::bind(&AnymalGazeboPlugin::actuatorCommandsCallback, this, std::placeholders::_1), *nodeHandle_);

  optionsActuatorCommands->autoSubscribe_ = false;
  optionsActuatorCommands->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  optionsActuatorCommands->tryRosResubscribing_ = false;

  actuatorCommandsSubscriber_ = cosmo_ros::subscribeShmRos<ActuatorCommandsShm, ActuatorCommandsRos, SeCommandConversionTrait>(
      "actuator_commands", optionsActuatorCommands);
}

void AnymalGazeboPlugin::initPublishers() {
  MELO_INFO_STREAM("Initializing Publishers");

  // Actuator readings.
  if (simulateEstimator_) {
    auto actReadingOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_readings_estimator", *nodeHandle_);
    actReadingOptions->rosQueueSize_ = 10u;
    actReadingOptions->rosLatch_ = false;
    actReadingOptions->autoPublishRos_ = false;

    actuatorReadingsPublisher_ = cosmo_ros::advertiseShmRos<ActuatorReadingsShm, ActuatorReadingsRos, SeReadingConversionTrait>(
        "actuator_readings_estimator", actReadingOptions);

  } else {
    auto actReadingOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_readings", *nodeHandle_);
    actReadingOptions->rosQueueSize_ = 10u;
    actReadingOptions->rosLatch_ = false;
    actReadingOptions->autoPublishRos_ = false;

    actuatorReadingsPublisher_ = cosmo_ros::advertiseShmRos<ActuatorReadingsShm, ActuatorReadingsRos, SeReadingConversionTrait>(
        "actuator_readings", actReadingOptions);
  }

  // Actuator readings throttled
  actuatorReadingsExtendedThrottledPublisher_ = nodeHandle_->advertise<anydrive_msgs::ReadingsExtended>(
      param_io::param<std::string>(*nodeHandle_, "publishers/actuator_readings_extended_throttled/topic", "/default"),
      param_io::param<unsigned int>(*nodeHandle_, "publishers/actuator_readings_extended_throttled/queue_size", 1u),
      param_io::param<bool>(*nodeHandle_, "publishers/actuator_readings_extended_throttled/latch", false));

  param_io::getParam(*nodeHandle_, "publishers/actuator_readings_extended_throttled/decimation",
                     actuatorReadingsExtendedThrottledDecimation_);

  // Contact forces
  if (simulateEstimator_) {
    std::string postFix = "_estimator";

    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      auto publishOptions =
          std::make_shared<cosmo_ros::PublisherRosOptions>(collisionLinkNamesToPublisherNames_[contactEnum] + postFix, *nodeHandle_);
      publishOptions->rosQueueSize_ = 1u;
      publishOptions->rosLatch_ = false;
      publishOptions->autoPublishRos_ = false;

      contactForcePublishers_[contactEnum] = cosmo_ros::advertiseShmRos<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>(
          collisionLinkNamesToPublisherNames_[contactEnum] + postFix, publishOptions);
    }
  } else {
    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      auto publishOptions =
          std::make_shared<cosmo_ros::PublisherRosOptions>(collisionLinkNamesToPublisherNames_[contactEnum], *nodeHandle_);
      publishOptions->rosQueueSize_ = 1u;
      publishOptions->rosLatch_ = false;
      publishOptions->autoPublishRos_ = false;

      contactForcePublishers_[contactEnum] = cosmo_ros::advertiseShmRos<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>(
          collisionLinkNamesToPublisherNames_[contactEnum], publishOptions);
    }
  }

  // Contact forces throttled
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    contactForcePublishersThrottled_[contactEnum] = nodeHandle_->advertise<geometry_msgs::WrenchStamped>(
        param_io::param<std::string>(*nodeHandle_, "publishers/" + collisionLinkNamesToPublisherNames_[contactEnum] + "_throttled/topic",
                                     "/default"),
        param_io::param<unsigned int>(*nodeHandle_,
                                      "publishers/" + collisionLinkNamesToPublisherNames_[contactEnum] + "_throttled/queue_size", 1u),
        param_io::param<bool>(*nodeHandle_, "publishers/" + collisionLinkNamesToPublisherNames_[contactEnum] + "_throttled/latch", false));

    contactForceThrottledDecimationMap_[contactEnum] = param_io::param<unsigned int>(
        *nodeHandle_, "publishers/" + collisionLinkNamesToPublisherNames_[contactEnum] + "_throttled/decimation", 40);
    contactForceThrottledCounterMap_[contactEnum] = 0;
  }

  if (simulateEstimator_) {
    // Joint States
    auto jointStateOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("joint_states", *nodeHandle_);
    jointStateOptions->rosQueueSize_ = 1u;
    jointStateOptions->rosLatch_ = false;
    jointStateOptions->autoPublishRos_ = false;
    jointStatesPublisher_ =
        cosmo_ros::advertiseShmRos<JointStateShm, JointStateRos, any_measurements_ros::ConversionTraits>("joint_states", jointStateOptions);

    // Anymal state.
    const std::string robotStateTopic = param_io::param<std::string>(*nodeHandle_, "publishers/anymal_state/topic", "/default");
    auto quadStateOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("anymal_state", *nodeHandle_);
    quadStateOptions->rosQueueSize_ = 1u;
    quadStateOptions->rosLatch_ = false;
    quadStateOptions->autoPublishRos_ = false;

    anymalStatePublisher_ =
        cosmo_ros::advertiseShmRos<AnymalStateShm, AnymalStateRos, anymal_model_ros::conversion_traits::ConversionTraits>("anymal_state",
                                                                                                                          quadStateOptions);
    // Anymal state throttled.
    anymalStateThrottledPublisher_ = nodeHandle_->advertise<anymal_msgs::AnymalState>(
        param_io::param<std::string>(*nodeHandle_, "publishers/anymal_state_throttle/topic", "/default"),
        param_io::param<unsigned int>(*nodeHandle_, "publishers/anymal_state_throttle/queue_size", 1u),
        param_io::param<bool>(*nodeHandle_, "publishers/anymal_state_throttle/latch", false));
    param_io::getParam(*nodeHandle_, "publishers/anymal_state_throttle/decimation", robotStateThrottledDecimation_);

    if (robotStateThrottledDecimation_ == 0) {
      ROS_WARN_STREAM_NAMED("gazebo_ros_control", "The anymal state throttle decimation must not be 0, setting it to 1.");
      robotStateThrottledDecimation_ = 1;
    }

    // Poses.
    auto odomOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("pose_in_odom", *nodeHandle_);
    odomOptions->rosQueueSize_ = 1u;
    odomOptions->rosLatch_ = false;
    odomOptions->autoPublishRos_ = false;

    poseInOdomPublisher_ = cosmo_ros::advertiseShmRos<PoseWithCovarianceShm, PoseWithCovarianceRos, any_measurements_ros::ConversionTraits>(
        "pose_in_odom", odomOptions);

    poseInOdomPublisherThrottled_ = nodeHandle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
        param_io::param<std::string>(*nodeHandle_, "publishers/pose_in_odom_throttled/topic", "/default"),
        param_io::param<unsigned int>(*nodeHandle_, "publishers/pose_in_odom_throttled/queue_size", 1u),
        param_io::param<bool>(*nodeHandle_, "publishers/pose_in_odom_throttled/latch", false));

    param_io::getParam(*nodeHandle_, "publishers/pose_in_odom_throttled/decimation", poseInOdomThrottledDecimation_);

    // Twist.
    auto twistOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("twist", *nodeHandle_);
    twistOptions->rosQueueSize_ = 1u;
    twistOptions->rosLatch_ = false;
    twistOptions->autoPublishRos_ = false;

    twistPublisher_ = cosmo_ros::advertiseShmRos<TwistShm, TwistRos, any_measurements_ros::ConversionTraits>("twist", twistOptions);

    twistPublisherThrottled_ = nodeHandle_->advertise<geometry_msgs::TwistWithCovarianceStamped>(
        param_io::param<std::string>(*nodeHandle_, "publishers/twist_throttled/topic", "/default"),
        param_io::param<unsigned int>(*nodeHandle_, "publishers/twist_throttled/queue_size", 1u),
        param_io::param<bool>(*nodeHandle_, "publishers/twist_throttled/latch", false));

    param_io::getParam(*nodeHandle_, "publishers/twist_throttled/decimation", twistThrottledDecimation_);
  } else if (publishGroundtruth_) {
    // Anymal state.
    auto quadStateOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("anymal_state_groundtruth", *nodeHandle_);
    quadStateOptions->rosQueueSize_ = 1u;
    quadStateOptions->rosLatch_ = false;
    quadStateOptions->autoPublishRos_ = false;
    anymalStatePublisher_ =
        cosmo_ros::advertiseShmRos<AnymalStateShm, AnymalStateRos, anymal_model_ros::conversion_traits::ConversionTraits>(
            "anymal_state_groundtruth", quadStateOptions);
  }
}

void AnymalGazeboPlugin::setActuatorGains() {
  for (const auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const std::string actuatorName = actuatorKey.getName();
    auto& model = actuatorModel_[actuatorEnum];

    double jointPositionPGain = 0.0;
    double jointPositionDGain = 0.0;
    double jointPositionVelocityTorquePGain = 0.0;
    double jointPositionVelocityTorqueIGain = 0.0;
    double jointPositionVelocityTorqueDGain = 0.0;

    if (actuatorName.find("HAA") != std::string::npos) {
      jointPositionPGain = param(*this->nodeHandle_, "actuators/gains/haa/jointPositionPGain", 100.0);
      jointPositionDGain = param(*this->nodeHandle_, "actuators/gains/haa/jointPositionDGain", 2.0);
      jointPositionVelocityTorquePGain = param(*this->nodeHandle_, "actuators/gains/haa/jointPositionVelocityTorquePGain", 30.0);
      jointPositionVelocityTorqueIGain = param(*this->nodeHandle_, "actuators/gains/haa/jointPositionVelocityTorqueIGain", 0.0);
      jointPositionVelocityTorqueDGain = param(*this->nodeHandle_, "actuators/gains/haa/jointPositionVelocityTorqueDGain", 0.0);
    } else if (actuatorName.find("HFE") != std::string::npos) {
      jointPositionPGain = param(*this->nodeHandle_, "actuators/gains/hfe/jointPositionPGain", 100.0);
      jointPositionDGain = param(*this->nodeHandle_, "actuators/gains/hfe/jointPositionDGain", 2.0);
      jointPositionVelocityTorquePGain = param(*this->nodeHandle_, "actuators/gains/hfe/jointPositionVelocityTorquePGain", 30.0);
      jointPositionVelocityTorqueIGain = param(*this->nodeHandle_, "actuators/gains/hfe/jointPositionVelocityTorqueIGain", 0.0);
      jointPositionVelocityTorqueDGain = param(*this->nodeHandle_, "actuators/gains/hfe/jointPositionVelocityTorqueDGain", 0.0);
    } else if (actuatorName.find("KFE") != std::string::npos) {
      jointPositionPGain = param(*this->nodeHandle_, "actuators/gains/kfe/jointPositionPGain", 100.0);
      jointPositionDGain = param(*this->nodeHandle_, "actuators/gains/kfe/jointPositionDGain", 2.0);
      jointPositionVelocityTorquePGain = param(*this->nodeHandle_, "actuators/gains/kfe/jointPositionVelocityTorquePGain", 30.0);
      jointPositionVelocityTorqueIGain = param(*this->nodeHandle_, "actuators/gains/kfe/jointPositionVelocityTorqueIGain", 0.0);
      jointPositionVelocityTorqueDGain = param(*this->nodeHandle_, "actuators/gains/kfe/jointPositionVelocityTorqueDGain", 0.0);
    } else {
      MELO_WARN_STREAM("Could not load actuator gains for actuator " << actuatorName);
    }

    model.getControllerFreeze().setJointPositionPGain(jointPositionPGain);
    model.getControllerFreeze().setJointPositionDGain(jointPositionDGain);
    model.getControllerJointPosition().setJointPositionPGain(jointPositionPGain);
    model.getControllerJointPosition().setJointPositionDGain(jointPositionDGain);
    model.getControllerJointPositionVelocity().setJointPositionPGain(jointPositionPGain);
    model.getControllerJointPositionVelocity().setJointPositionDGain(jointPositionDGain);
    model.getControllerJointPositionVelocityTorque().setJointPositionPGain(jointPositionVelocityTorquePGain);
    model.getControllerJointPositionVelocityTorque().setJointPositionIGain(jointPositionVelocityTorqueIGain);
    model.getControllerJointPositionVelocityTorque().setJointPositionDGain(jointPositionVelocityTorqueDGain);
  }
}

void AnymalGazeboPlugin::initServices() {
  // Set robot pose.
  setRobotPoseServer_ =
      nodeHandle_->advertiseService(param_io::param<std::string>(*nodeHandle_, "servers/set_robot_pose/service", "/default"),
                                    &AnymalGazeboPluginBase::setRobotPoseCb, static_cast<AnymalGazeboPluginBase*>(this));
}

void AnymalGazeboPlugin::publishActuatorReadingsDerived() {
  if (actuatorReadingsExtendedThrottledCounter_ == actuatorReadingsExtendedThrottledDecimation_) {
    if (actuatorReadingsExtendedThrottledPublisher_.getNumSubscribers() > 0u) {
      for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
        const auto actuatorEnum = actuatorKey.getEnum();
        const auto actuatorId = actuatorKey.getId();
        const auto& seaReading = actuatorReadings_[actuatorEnum];

        // Convert the SEA state to an ANYdrive state.
        anydrive::State state;
        series_elastic_actuator_anydrive::seActuatorToAnydrive(state, seaReading.getState());

        // Convert the SEA mode to an ANYdrive mode (separately as it is not included in the state, but only in the mode).
        anydrive::mode::ModeEnum modeEnum;
        series_elastic_actuator_anydrive::seActuatorToAnydrive(modeEnum, actuatorCommands_[actuatorEnum].getModeEnum());

        // Set the statusword.
        anydrive::Statusword statusword = state.getStatusword();
        statusword.setStateEnum(anydrive::fsm::StateEnum::ControlOp);
        statusword.setModeEnum(modeEnum);
        state.setStatusword(statusword);

        // Convert the state to an extended state.
        anydrive::StateExtended stateExtended;
        static_cast<anydrive::State&>(stateExtended) = state;

        // Create an extended reading.
        anydrive::ReadingExtended readingExtended;
        readingExtended.setState(stateExtended);
        series_elastic_actuator_anydrive::seActuatorToAnydrive(readingExtended.getCommanded(), seaReading.getCommanded());

        // Convert to a ROS message.
        anydrive_ros::writeToMessage(readingsExtendedMsg_.readings.at(actuatorId), readingExtended);
      }
      actuatorReadingsExtendedThrottledPublisher_.publish(readingsExtendedMsg_);
    }
    actuatorReadingsExtendedThrottledCounter_ = 0;
  }
  ++actuatorReadingsExtendedThrottledCounter_;
}

void AnymalGazeboPlugin::publishRobotState() {
  any_measurements::Time stamp = getTime();
  anymalMsgShm_.time_ = stamp;
  anymalMsgShm_.status_ = anymal_model::StateStatus::STATUS_OK;

  for (const auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    anymalMsgShm_.anymalState_.getJointPositions()(actuatorId) = actuatorReadings_[actuatorEnum].getState().getJointPosition();
    anymalMsgShm_.anymalState_.getJointVelocities()(actuatorId) = actuatorReadings_[actuatorEnum].getState().getJointVelocity();
    anymalMsgShm_.anymalState_.getJointAccelerations()(actuatorId) = actuatorReadings_[actuatorEnum].getState().getJointAcceleration();
    anymalMsgShm_.anymalState_.getJointTorques()(actuatorId) = actuatorReadings_[actuatorEnum].getState().getJointTorque();
  }

#if (GAZEBO_MAJOR_VERSION >= 8)
  const kindr::RotationQuaternionD orientationBaseToWorld(robotBaseLinkPose_.Rot().W(), robotBaseLinkPose_.Rot().X(),
                                                          robotBaseLinkPose_.Rot().Y(), robotBaseLinkPose_.Rot().Z());
#else
  const kindr::RotationQuaternionD orientationBaseToWorld(robotBaseLinkPose_.rot.w, robotBaseLinkPose_.rot.x, robotBaseLinkPose_.rot.y,
                                                          robotBaseLinkPose_.rot.z);
#endif

  const kindr::Velocity3D B_v_B(robotBaseLinkLinearVelocity_[0], robotBaseLinkLinearVelocity_[1], robotBaseLinkLinearVelocity_[2]);

  const kindr::LocalAngularVelocityPD B_w_IB(robotBaseLinkAngularVelocity_[0], robotBaseLinkAngularVelocity_[1],
                                             robotBaseLinkAngularVelocity_[2]);

  //-- Generalized positions
#if (GAZEBO_MAJOR_VERSION >= 8)
  anymalMsgShm_.anymalState_.setPositionWorldToBaseInWorldFrame(
      anymal_model::Position(robotBaseLinkPose_.Pos().X() - frameOdometryOffsetX_, robotBaseLinkPose_.Pos().Y() - frameOdometryOffsetY_,
                             robotBaseLinkPose_.Pos().Z() - frameOdometryOffsetZ_));
#else
  anymalMsgShm_.anymalState_.setPositionWorldToBaseInWorldFrame(anymal_model::Position(robotBaseLinkPose_.pos[0] - frameOdometryOffsetX_,
                                                                                       robotBaseLinkPose_.pos[1] - frameOdometryOffsetY_,
                                                                                       robotBaseLinkPose_.pos[2] - frameOdometryOffsetZ_));
#endif
  anymalMsgShm_.anymalState_.setOrientationBaseToWorld(orientationBaseToWorld);
  //--

  //-- Generalized velocities
  anymalMsgShm_.anymalState_.setLinearVelocityBaseInWorldFrame(orientationBaseToWorld.rotate(B_v_B));
  anymalMsgShm_.anymalState_.setAngularVelocityBaseInBaseFrame(B_w_IB);

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    auto& contact = anymalMsgShm_.contacts_[contactEnum];
    contact.time_ = stamp;

    auto normal = contactNormals_[contactEnum];
    contact.normal_.x() = normal[0];
    contact.normal_.y() = normal[1];
    contact.normal_.z() = normal[2];

    auto position = contactPositions_[contactEnum];
    contact.position_.x() = position[0] - frameOdometryOffsetX_;
    contact.position_.y() = position[1] - frameOdometryOffsetY_;
    contact.position_.z() = position[2] - frameOdometryOffsetZ_;

    auto force = contactForces_[contactEnum];
    contact.wrench_.wrench_.getForce().x() = force[0];
    contact.wrench_.wrench_.getForce().y() = force[1];
    contact.wrench_.wrench_.getForce().z() = force[2];

    auto torque = contactTorques_[contactEnum];
    contact.wrench_.wrench_.getTorque().x() = torque[0];
    contact.wrench_.wrench_.getTorque().y() = torque[1];
    contact.wrench_.wrench_.getTorque().z() = torque[2];

    if (contactFlags_[contactEnum]) {
      const double normXY = std::sqrt(
#if (GAZEBO_MAJOR_VERSION >= 8)
          contactLinearVelocities_[contactEnum].X() * contactLinearVelocities_[contactEnum].X() +
          contactLinearVelocities_[contactEnum].Y() * contactLinearVelocities_[contactEnum].Y());
#else
          contactLinearVelocities_[contactEnum].x * contactLinearVelocities_[contactEnum].x +
          contactLinearVelocities_[contactEnum].y * contactLinearVelocities_[contactEnum].y);
#endif

      if (normXY < contactLinearVelocityThreshold_) {
        contact.state_ = static_cast<unsigned int>(AD::ContactStateEnum::CLOSED);
      } else {
        contact.state_ = static_cast<unsigned int>(AD::ContactStateEnum::SLIPPING);
      }
    } else {
      contact.state_ = static_cast<unsigned int>(AD::ContactStateEnum::OPEN);
    }
  }

  // tfs
  // update anymal model
  anymalModel_.setState(anymalMsgShm_.anymalState_, true, false, false);

  for (const auto contactKeys : AD::getContactKeys()) {
    const auto contactEnum = contactKeys.getEnum();

    switch (anymalMsgShm_.contacts_[contactEnum].state_) {
      case static_cast<unsigned int>(AD::ContactStateEnum ::OPEN):
        anymalModel_.getContactContainer()[contactEnum]->setState(anymal_description::AnymalTopology::ContactStateEnum::OPEN);
        break;
      case static_cast<unsigned int>(AD::ContactStateEnum ::CLOSED):
        anymalModel_.getContactContainer()[contactEnum]->setState(anymal_description::AnymalTopology::ContactStateEnum::CLOSED);
        break;
      case static_cast<unsigned int>(AD::ContactStateEnum ::SLIPPING):
        anymalModel_.getContactContainer()[contactEnum]->setState(anymal_description::AnymalTopology::ContactStateEnum::SLIPPING);
        break;
      default:
        break;
    }
  }

  if (!(simulateEstimator_ || publishGroundtruth_)) {
    return;
  }

  // update frames generators
  framesGenerator_.update(anymalModel_);

  kindr::HomTransformQuatD transform;

  transform.setIdentity();
  transform = framesGenerator_.getPoseFootprintToOdom();
  anymalMsgShm_.anymalState_.setFrameTransform(anymal_description::AnymalTopology::FrameTransformEnum::FootprintToOdom, transform);

  transform.setIdentity();
  transform = framesGenerator_.getPoseFeetcenterToOdom();
  anymalMsgShm_.anymalState_.setFrameTransform(anymal_description::AnymalTopology::FrameTransformEnum::FeetcenterToOdom, transform);

  anymal_model_ros::toRos(anymalMsgShm_, anymalMsgRos_);
  anymalStatePublisher_->publish(anymalMsgShm_, anymalMsgRos_, std::chrono::microseconds(200));
}

AnymalGazeboPlugin::WrenchShm AnymalGazeboPlugin::createContactWrenchMessage(const ContactEnum& contactEnum) {
  any_measurements::Wrench contactWrench;
  contactWrench.time_ = getTime();
  const auto& force = contactForces_[contactEnum];
  const auto& torque = contactTorques_[contactEnum];
  kindr::Force3D forceInWorld(force[0], force[1], force[2]);
  kindr::Torque3D torqueInWorld(torque[0], torque[1], torque[2]);

  if (!simulateEstimator_) {
    const auto bodyEnum = AD::mapEnums<AD::BodyEnum>(contactEnum);
    kindr::RotationMatrixD rotWorldToBase(anymalModel_.getOrientationWorldToBody(bodyEnum));
    contactWrench.wrench_.getForce() = rotWorldToBase.rotate(forceInWorld);
    contactWrench.wrench_.getTorque() = rotWorldToBase.rotate(torqueInWorld);
  } else {
    contactWrench.wrench_.getForce() = forceInWorld;
    contactWrench.wrench_.getTorque() = torqueInWorld;
  }

  return contactWrench;
}

void AnymalGazeboPlugin::sendRos() {
  actuatorReadingsPublisher_->sendRos();

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    contactForcePublishers_[contactEnum]->sendRos();
  }

  if (simulateEstimator_) {
    anymalStatePublisher_->sendRos();  // visualization now with cosmo

    if (robotStateThrottledCounter_ == robotStateThrottledDecimation_) {
      if (anymalStateThrottledPublisher_.getNumSubscribers() > 0u) {
        anymalStateThrottledPublisher_.publish(
            anymal_model_ros::toRos<anymal_model::ExtendedAnymalState, anymal_msgs::AnymalState>(anymalMsgShm_));
      }
      robotStateThrottledCounter_ = 0;
    }
    ++robotStateThrottledCounter_;

    poseInOdomPublisher_->sendRos();
    twistPublisher_->sendRos();

    jointStatesPublisher_->sendRos();
  } else if (publishGroundtruth_) {
    anymalStatePublisher_->sendRos();
  }
}

GZ_REGISTER_MODEL_PLUGIN(AnymalGazeboPlugin)
}  // namespace gazebo
