#include <anymal_state_estimator_lwf/anymal_filter_lwf/AnymalFilterLwf.hpp>

namespace anymal_state_estimator_lwf {

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::AnymalFilterLwf(
    any_node::Node::NodeHandlePtr nh)
    : Base(nh),
      filterKinematicModelPtr_(nullptr),
      odometryToBaseOutputCT_(std::get<1>(filter_.mUpdates_)) {}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::~AnymalFilterLwf() {
  delete filterKinematicModelPtr_;
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
bool AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::initFilter(
    const kindr::HomTransformQuatD& poseBaseToOdom) {
  // Initialize the anymal model
  MELO_INFO_STREAM("[AnymalFilterLwf] Initializing anymal model.");

  filterKinematicModelPtr_ = new LwfModelType_();
  std::string anymalUrdfDescription = param_io::param<std::string>(*this->nh_, "/anymal_description", "");

  if (!filterKinematicModelPtr_->initializeFromUrdf(anymalUrdfDescription)) {
    MELO_FATAL_STREAM("Could not load model from URDF! " << anymalUrdfDescription);
    return false;
  }

  std::get<1>(filter_.mUpdates_).setModelPtr(filterKinematicModelPtr_);

  auto jacobianTestDelta = param_io::param<double>(*this->nh_, "lwf/jacobian_test_delta", 1e-6);
  MELO_INFO_STREAM("[AnymalFilterLwf] jacobian_test_delta is " << jacobianTestDelta);

  std::string filterPath =
      param_io::param<std::string>(*this->nh_, "lwf/filter_config_file", "/default/path/to/filters/");
  MELO_INFO_STREAM("[AnymalFilterLwf::initialize] Using filter configuration from file: '" << filterPath << "'");

  filter_.readFromInfo(filterPath);
  filter_.safe_ = filter_.init_;
  filter_.logCountDiagnostics_ = false;
  filter_.mPrediction_.testJacs();
  MELO_INFO("Test update jacobian 0");
  std::get<0>(filter_.mUpdates_).testJacs(jacobianTestDelta);
  MELO_INFO("Test update jacobian 1");
  std::get<1>(filter_.mUpdates_).testJacs(jacobianTestDelta);
  std::get<0>(filter_.updateTimelineTuple_).maxWaitTime_ = 0.0;
  std::get<1>(filter_.updateTimelineTuple_).maxWaitTime_ = 0.1;

  MELO_INFO("Test odometry output");
  odometryToBaseOutputCT_.testTransformJac(jacobianTestDelta);
  MELO_INFO("Test standard pose output");
  transformStandardPoseCF_.testTransformJac(jacobianTestDelta);

  // Publish latched message
  return resetFilter(poseBaseToOdom);
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
bool AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::resetFilter(
    const kindr::HomTransformQuatD& poseBaseToOdom) {
  Eigen::Vector3d IrIB = poseBaseToOdom.getPosition().toImplementation();
  rot::RotationQuaternionPD qBI = poseBaseToOdom.getRotation().inverted();

  if (qBI.norm() == 0) {
    qBI.setIdentity();
  } else {
    qBI.fix();
  }

  filter_.resetToBodyPose(IrIB, qBI, ros::Time::now().toSec());

  isInitialized_ = false;
  MELO_INFO_STREAM("[AnymalFilterLwf] Filter was uninitialized.");

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
void AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::processImuReadings(
    const any_measurements::Imu& imu) {
  const auto imuStamp = imu.time_;
  if (imuStamp.toSeconds() <= prevMeasImuStamp_.toSeconds()) {
    return;
  }

  prevMeasImuStamp_ = imuStamp;

  mtPredictionMeas predictionMeas;
  predictionMeas.template get<mtPredictionMeas::_acc>() = imu.linearAcceleration_.toImplementation();

  predictionMeas.template get<mtPredictionMeas::_gyr>() =
      0.5 * (imu.angularVelocity_.toImplementation() + this->previousAngularVelocityMeasurement_);
  this->previousAngularVelocityMeasurement_ = imu.angularVelocity_.toImplementation();

  if (!isInitialized_) {
    // if(this->hasFullContact_) {
    {
      filter_.resetWithAccelerometer(predictionMeas.template get<mtPredictionMeas::_acc>(), imuStamp.toSeconds());
      MELO_INFO_STREAM("predictionMeas" << predictionMeas.template get<mtPredictionMeas::_acc>());
      MELO_INFO_STREAM("time seconds: " << imuStamp.toSeconds());
    }

    isInitialized_ = true;
    MELO_INFO("============> State estimator: Initialized filter!");
    // }
  } else {
    filter_.addPredictionMeas(predictionMeas, imuStamp.toSeconds());
  }
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
void AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::processKinematics(
    const JointState& joints, const ContactEnumContainer<Contact>& contacts, const bool fakeKinematicsUpdateActive) {
  const auto jointStamp = joints.front().time_;
  mtKinMeas kinUpdateMeas;

  for (auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto contactId = contactKey.getId();

    kinUpdateMeas.template get<mtKinMeas::_enc>(contactId) =
        Eigen::Vector3d(joints[RD::template mapKeyIdToKeyEnum<JointEnum>(3 * contactId + 0)].position_,
                        joints[RD::template mapKeyIdToKeyEnum<JointEnum>(3 * contactId + 1)].position_,
                        joints[RD::template mapKeyIdToKeyEnum<JointEnum>(3 * contactId + 2)].position_);
    kinUpdateMeas.template get<mtKinMeas::_end>(contactId) =
        Eigen::Vector3d(joints[RD::template mapKeyIdToKeyEnum<JointEnum>(3 * contactId + 0)].velocity_,
                        joints[RD::template mapKeyIdToKeyEnum<JointEnum>(3 * contactId + 1)].velocity_,
                        joints[RD::template mapKeyIdToKeyEnum<JointEnum>(3 * contactId + 2)].velocity_);
    // estimate contact forces when a joint state is received
    kinUpdateMeas.template get<mtKinMeas::_aux>().contactFlags_(contactId) =
        contacts[contactEnum].flag_ || fakeKinematicsUpdateActive;
  }

  filter_.template addUpdateMeas<1>(kinUpdateMeas, jointStamp.toSeconds());
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
void AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::updateFilter() {
  // TODO: Get safe time??
  this->lastEstimatedStateStamp_ = any_measurements_ros::fromRos(ros::Time::now());

  if (isInitialized_) {
    filter_.updateSafe();
  }

  // ODOMETRY -> BASE
  odometryToBaseOutputCT_.transformState(filter_.safe_.state_, odometryOutput_);
  odometryToBaseOutputCT_.transformCovMat(filter_.safe_.state_, filter_.safe_.cov_, odometryOutputCov_);
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
typename AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::StateStatus
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getStatus() const {
  StateStatus status;
  const auto stampNow = any_measurements_ros::fromRos(ros::Time::now());

  if (isInitialized_) {
    // always set pose sensor warning since the LWF never processes localizer messages
    status = StateStatus::STATUS_OK;
  } else {
    status = StateStatus::STATUS_ERROR_UNKNOWN;
  }

  return status;
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
any_measurements::Time AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getLastImuStamp() const {
  return prevMeasImuStamp_;
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
kindr::RotationQuaternionPD
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getOrientationOdomToBase() const {
  return odometryOutput_.template get<mtOutput::_att>();
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
kindr::Position3D
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getPositionWorldToBaseInWorldFrame() const {
  return kindr::Position3D(odometryOutput_.template get<mtOutput::_pos>());
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
kindr::Velocity3D AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getLinearVelocityInBaseFrame()
    const {
  return kindr::Velocity3D(odometryOutput_.template get<mtOutput::_vel>());
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
kindr::LocalAngularVelocityPD
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getAngularVelocityBaseInBaseFrame() const {
  return kindr::LocalAngularVelocityPD(odometryOutput_.template get<mtOutput::_ror>());
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
Eigen::Matrix<double, 3, 1>
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getImuLinearAccelerationBias() const {
  return filter_.safe_.state_.template get<mtFilter::mtState::_gyb>();
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
Eigen::Matrix<double, 3, 1>
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getImuAngularVelocityBias() const {
  return filter_.safe_.state_.template get<mtFilter::mtState::_acb>();
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
typename AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::PoseCovarianceMatrix
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getEstPoseInOdomCovariance() const {
  PoseCovarianceMatrix output;
  output.template topLeftCorner<3, 3>() = odometryOutputCov_.block<3, 3>(mtOutput::template getId<mtOutput::_pos>(),
                                                                         mtOutput::template getId<mtOutput::_pos>());
  output.template topRightCorner<3, 3>() = odometryOutputCov_.block<3, 3>(mtOutput::template getId<mtOutput::_pos>(),
                                                                          mtOutput::template getId<mtOutput::_att>());
  output.template bottomLeftCorner<3, 3>() = odometryOutputCov_.block<3, 3>(mtOutput::template getId<mtOutput::_att>(),
                                                                            mtOutput::template getId<mtOutput::_pos>());
  output.template bottomRightCorner<3, 3>() = odometryOutputCov_.block<3, 3>(
      mtOutput::template getId<mtOutput::_att>(), mtOutput::template getId<mtOutput::_att>());
  return output;
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
typename AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::PoseCovarianceMatrix
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getEstTwistInBaseCovariance() const {
  PoseCovarianceMatrix output;
  output.template topLeftCorner<3, 3>() = alignedPoseOutputCov_.block<3, 3>(mtOutput::template getId<mtOutput::_vel>(),
                                                                            mtOutput::template getId<mtOutput::_vel>());
  output.template topRightCorner<3, 3>() = alignedPoseOutputCov_.block<3, 3>(
      mtOutput::template getId<mtOutput::_vel>(), mtOutput::template getId<mtOutput::_ror>());
  output.template bottomLeftCorner<3, 3>() = alignedPoseOutputCov_.block<3, 3>(
      mtOutput::template getId<mtOutput::_ror>(), mtOutput::template getId<mtOutput::_vel>());
  output.template bottomRightCorner<3, 3>() = alignedPoseOutputCov_.block<3, 3>(
      mtOutput::template getId<mtOutput::_ror>(), mtOutput::template getId<mtOutput::_ror>());
  return output;
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
kindr::RotationQuaternionPD
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getOrientationBaseToWorld() const {
  return getOrientationOdomToBase().inverted();
}

template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
kindr::Velocity3D
AnymalFilterLwf<ConcreteDescription_, RobotState_, LwfModelType_>::getLinearVelocityBaseInWorldFrame() const {
  const auto orientationBaseToWorld = getOrientationBaseToWorld();
  return orientationBaseToWorld.rotate(getLinearVelocityInBaseFrame());
}

}  // namespace anymal_state_estimator_lwf