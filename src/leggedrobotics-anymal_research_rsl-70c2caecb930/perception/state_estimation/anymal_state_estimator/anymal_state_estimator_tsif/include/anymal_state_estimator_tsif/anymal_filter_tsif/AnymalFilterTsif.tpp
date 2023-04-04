#include <anymal_state_estimator_tsif/anymal_filter_tsif/AnymalFilterTsif.hpp>

namespace anymal_state_estimator_tsif {

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::AnymalFilterTsif(any_node::Node::NodeHandlePtr nh)
    : Base(nh),
      odometryFilter_(*nh),
      previousLinearAccelerationMeasurement_(tsif::Vec3::Zero()),
      imuToBaseRotationMatrix_(tsif::Mat3::Identity()),
      baseToImuPosition_(tsif::Vec3::Zero()),
      gyroBiasEstimationContactCountThreshold_(param_io::param<int>(*nh, "tsif/gyro_bias_estimation_contact_count_threshold", 5)),
      accelerometerBiasEstimationContactCountThreshold_(param_io::param<int>(*nh, "tsif/accelerometer_bias_estimation_contact_count_threshold", 5)),
      initialActuatorReadingsSet_(false),
      imuInitializationFinished_(false),
      odometryFilterReadyToUpdate_(false),
      imuInitializer_(param_io::param<int>(*nh, "tsif/n_init_imu_measurements", 100),
                      param_io::param<int>(*nh, "tsif/n_throwaway_init_imu_measurements", 20),
                      param_io::param<int>(*nh, "tsif/velocity_check_history_size", 200),
                      param_io::param<double>(*nh, "tsif/linear_variance_threshold", 0.01),
                      param_io::param<double>(*nh, "tsif/angular_variance_threshold", 0.005)) {
  odometryStateCovariance_.setIdentity();
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::readParameters() {
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::addVariablesToLog() {
  signal_logger::add(imuLinearAccelerationBiasCovarianceDiagonal_,
                     std::string{"imuLinearAccelerationBiasCovarianceDiagonal"});
  signal_logger::add(imuAngularVelocityBiasCovarianceDiagonal_,
                     std::string{"imuAngularVelocityBiasCovarianceDiagonal"});
  signal_logger::add(imuInitializer_.getMeanLinAccWorldToImuInBaseFrame(), "meanLinAccWorldToImuInBaseFrame");
  signal_logger::add(imuInitializer_.getVarLinAccWorldToImuInBaseFrame(), "varLinAccWorldToImuInBaseFrame");
  signal_logger::add(imuInitializer_.getVarLinAccWorldToImuInBaseFrameThreshold(), "varLinAccWorldToImuInBaseFrameThreshold");
  signal_logger::add(imuInitializer_.getMeanAngAccWorldToImuInBaseFrame(), "meanAngVelWorldToBaseInBaseFrame");
  signal_logger::add(imuInitializer_.getVarAngAccWorldToImuInBaseFrame(), "varAngVelWorldToBaseInBaseFrame");
  signal_logger::add(imuInitializer_.getVarAngVelWorldToBaseInBaseFrameThreshold(), "varAngVelWorldToBaseInBaseFrameThreshold");
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
bool AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::initFilter(
    const kindr::HomTransformQuatD& poseBaseToOdom) {

  // Initialization Guards.
  std::string imu_link;
  if(!param_io::getParam(*this->nh_, "imu_frame_id", imu_link)) {
    return false;
  } 
  if(!imuLinkAvailable(imu_link)) {
    MELO_WARN("IMU link '%s' is not available in rbdl model.", imu_link.c_str());
    return false;
  }

  MELO_INFO("State estimator is setting up.");

  readParameters();

  bool printDiagnostics = param_io::param<bool>(*this->nh_, "tsif/print_diagnostics", false);

  // get imu offset parameters from model or parameters files
  RigidBodyDynamics::Model& rbdlModel =
      std::static_pointer_cast<romo_rbdl::RobotModelRbdl<ConcreteDescription_, RobotState_>>(robotModelPtr_)->getRbdlModel();
  const unsigned int imuBodyId = rbdlModel.GetBodyId(imu_link.c_str());

  baseToImuPosition_ = rbdlModel.mFixedBodies[imuBodyId - rbdlModel.fixed_body_discriminator]
                             ->mParentTransform.r;  // r = _{Parent}r_{ParentChild}
  imuToBaseRotationMatrix_ = rbdlModel.mFixedBodies[imuBodyId - rbdlModel.fixed_body_discriminator]
                                   ->mParentTransform.E.inverse();  // E = R_{ChildParent}

  // set imu offset via setter (not parameters) since it is supposed to be determined from the model
  odometryFilter_.SetImuOffset(baseToImuPosition_);
  if (printDiagnostics) {
    MELO_INFO_STREAM("Odometry filter Connectivity:" << odometryFilter_.PrintConnectivity());
    MELO_INFO("Odometry filter jacobian tests:");
    odometryFilter_.JacTestAll(1e-5, 1e-8);
  }

  MELO_INFO("State estimator is set up.");

  return resetFilter(poseBaseToOdom);
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
bool AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::imuLinkAvailable(
    const std::string& imu_link) {
  RigidBodyDynamics::Model& rbdlModel =
      std::static_pointer_cast<romo_rbdl::RobotModelRbdl<ConcreteDescription_, RobotState_>>(robotModelPtr_)->getRbdlModel();

  return rbdlModel.GetBodyId(imu_link.c_str()) != std::numeric_limits<unsigned int>::max();
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
bool AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::resetFilter(
    const kindr::HomTransformQuatD& poseBaseToOdom) {
  //reset odometry filter with pose
  odometryFilter_.SetInitPose(poseBaseToOdom);
  odometryFilter_.Uninitialize();
  odometryFilter_.Clear();
  initialActuatorReadingsSet_ = false;
  imuInitializationFinished_ = false;
  odometryFilterReadyToUpdate_ = false;
  imuInitializer_.reset();
  MELO_INFO("Odom filter was uninitialized.");
  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::processImuReadings(
    const any_measurements::Imu& imu) {
  tsif::Vec3 measLinAccWorldToImuInBaseFrame;
  tsif::Vec3 measAngVelWorldToBaseInBaseFrame;
  const auto measurementTime = imu.time_.toSeconds();

  measLinAccWorldToImuInBaseFrame =
      0.5 * imuToBaseRotationMatrix_ *
      (imu.linearAcceleration_.toImplementation() + previousLinearAccelerationMeasurement_);
  previousLinearAccelerationMeasurement_ = imu.linearAcceleration_.toImplementation();
  measAngVelWorldToBaseInBaseFrame =
      0.5 * imuToBaseRotationMatrix_ *
      (imu.angularVelocity_.toImplementation() + this->previousAngularVelocityMeasurement_);
  this->previousAngularVelocityMeasurement_ = imu.angularVelocity_.toImplementation();

  // //sum up measurements as a quick measurement sanity check
  const double checksum = measLinAccWorldToImuInBaseFrame.sum() + measAngVelWorldToBaseInBaseFrame.sum();
  const bool badMeasurement = std::isnan(checksum) || std::isinf(checksum) || (measurementTime <= 0.);

  if (badMeasurement) {
    MELO_WARN_THROTTLE(1., "Bad IMU reading! (Warning is throttled: 1s)");
    return;
  }

  if (odometryFilterReadyToUpdate_) {
    odometryFilter_.AddMeasurementLinearAccelerationWorldToImuInBase(tsif::TimePoint(tsif::fromSec(measurementTime)),
                                                                     measLinAccWorldToImuInBaseFrame);
    odometryFilter_.AddMeasurementAngularVelocityWorldToBaseInBase(tsif::TimePoint(tsif::fromSec(measurementTime)),
                                                                   measAngVelWorldToBaseInBaseFrame);
  }

  // if the filter is not updating yet, update the pose and bias initializer and try to set the initial pose/bias
  if (!odometryFilter_.IsInitialized()) {
    imuInitializer_.addMeasurement(measurementTime, measLinAccWorldToImuInBaseFrame, measAngVelWorldToBaseInBaseFrame);
    if (!imuInitializationFinished_) {
      imuInitializationFinished_ = imuInitializer_.overrideBaseToWorldRollAndPitch(odometryFilter_.GetInitPose().getRotation()) && imuInitializer_.getGyroBiasInBase(odometryFilter_.GetInitGyroBias());
      if (imuInitializationFinished_) {
        MELO_INFO_STREAM("Initializing with gyro bias = " << odometryFilter_.GetInitGyroBias().transpose());
      }
    }
  }
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::processJointStates(const JointState& joints,
    const bool fakeKinematicsUpdateActive) {
  const auto measurementTime = joints.front().time_.toSeconds();

  double checksum = 0.;
  // get footcenter to contact correction term
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    // stop updating measuredFootholds_ in case of fake kinematics
    // TODO: scale landmark update weight depending on leg configuration
    if (!fakeKinematicsUpdateActive) {
      measuredFootholds_[contactEnum] =
          robotModelPtr_->getPositionBodyToBody(BodyEnum::BASE,
                                                RD::template mapKeyEnumToKeyEnum<ContactEnum, BodyEnum>(contactEnum),
                                                RD::CoordinateFrameEnum::BASE);
    }
    // sum up measurements as a quick measurement sanity check
    checksum += measuredFootholds_[contactEnum].sum();
  }

  bool badMeasurement = std::isnan(checksum) || std::isinf(checksum) || (measurementTime <= 0.);

  if (badMeasurement) {
    MELO_WARN_THROTTLE(1., "Bad actuator readings! (Warning is throttled: 1s)");
    return;
  }

  if (odometryFilterReadyToUpdate_) odometryFilter_.AddMeasurementPositionsBaseToFootInBase(tsif::TimePoint(tsif::fromSec(measurementTime)),
                                                                                            measuredFootholds_);

  if (!initialActuatorReadingsSet_ && this->initialFullContactSet_) {
    odometryFilter_.SetInitContacts(measuredFootholds_);
    initialActuatorReadingsSet_ = true;
  }
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::processContacts(
    const ContactEnumContainer<Contact>& contacts, const bool fakeKinematicsUpdateActive) {
  bool badMeasurement = false;

  unsigned int contactCount = 0u;
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    if (contacts[contactEnum].stamp_.toSeconds() <= 0.) {
      badMeasurement = true;
    }
    if (contacts[contactEnum].flag_) {
      contactCount++;
    }
  }

  if (badMeasurement) {
    MELO_WARN_THROTTLE(1., "Bad contact flags! (Warning is throttled: 1s)");
    return;
  }

  if (odometryFilterReadyToUpdate_) {
    odometryFilter_.AddMeasurementContacts(contacts, fakeKinematicsUpdateActive);
    odometryFilter_.ToggleGyroBiasEstimation(contactCount>=gyroBiasEstimationContactCountThreshold_);
    odometryFilter_.ToggleAccelerometerBiasEstimation(contactCount>=accelerometerBiasEstimationContactCountThreshold_); 
  }
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::processKinematics(
    const JointState& joints, const ContactEnumContainer<Contact>& contacts, const bool fakeKinematicsUpdateActive) {
  processJointStates(joints, fakeKinematicsUpdateActive);
  processContacts(contacts, fakeKinematicsUpdateActive);
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::updateFilter() {
  this->lastEstimatedStateStamp_ = any_measurements_ros::fromRos(ros::Time::now());

  // check if filters are ready to update
  odometryFilterReadyToUpdate_ = initialActuatorReadingsSet_ && imuInitializationFinished_;

  if (odometryFilterReadyToUpdate_) odometryFilter_.Update();

  setOutput();
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
void AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::setOutput() {

  if (odometryFilter_.IsInitialized()) {
    odometryStateCovariance_ = odometryFilter_.GetCovariance();
    prevMeasImuStamp_ = any_measurements::Time(odometryFilter_.GetLastImuStamp());
    estPositionWorldToBaseInWorld_ = odometryFilter_.GetPositionWorldToBaseInWorld();
    estOrientationBaseToWorld_ = odometryFilter_.GetOrientationBaseToWorld();
    estLinVelWorldToImuInBase_ = odometryFilter_.GetLinearVelocityWorldToImuInBase();
    estAngVelWorldToBaseInBase_ = odometryFilter_.GetAngularVelocityWorldToBaseInBase();
    imuLinearAccBias_ = imuToBaseRotationMatrix_.transpose() * odometryFilter_.GetLinearAccelerationBiasInBase();
    imuAngularVelBias_ = imuToBaseRotationMatrix_.transpose() * odometryFilter_.GetAngularVelocityBiasInBase();
  } else {
    odometryStateCovariance_.setIdentity();
    estPositionWorldToBaseInWorld_.setZero();
    estOrientationBaseToWorld_.setIdentity();
    estLinVelWorldToImuInBase_.setZero();
    estAngVelWorldToBaseInBase_.setZero();
    imuLinearAccBias_.setZero();
    imuAngularVelBias_.setZero();
  }

  bool odometryStateError = false;
  double odometryChecksum = estOrientationBaseToWorld_.norm() + estPositionWorldToBaseInWorld_.sum() +
                            estLinVelWorldToImuInBase_.sum() + estAngVelWorldToBaseInBase_.sum() +
                            odometryStateCovariance_.sum();
  if (std::isnan(odometryChecksum) || std::isinf(odometryChecksum)) {
    odometryStateError = true;
  }
  if (odometryStateError) {
    MELO_WARN_THROTTLE(1., "State and/or covariance of odom filter contains NaN or Inf. (Warning is throttled: 1s)");
  }

  //set quantities for the signal logger
  const auto imuLinAccBiasIndex = odometryFilter_.GetLinearAccelerationBiasInBaseIndex();
  const auto imuAngVelBiasIndex = odometryFilter_.GetAngularVelocityBiasInBaseIndex();
  for (int i=0; i<3; i++){
    imuLinearAccelerationBiasCovarianceDiagonal_(i) = odometryStateCovariance_.block<3,3>(imuLinAccBiasIndex,imuLinAccBiasIndex)(i,i);
    imuAngularVelocityBiasCovarianceDiagonal_(i) = odometryStateCovariance_.block<3,3>(imuAngVelBiasIndex,imuAngVelBiasIndex)(i,i);
  }

}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
typename AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::StateStatus
AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getStatus() const {
  StateStatus status = StateStatus::STATUS_OK;

  if (!odometryFilter_.IsInitialized()) {
    return StateStatus::STATUS_ERROR_UNKNOWN;
  }

  return status;
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
kindr::Position3D AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getPositionWorldToBaseInWorldFrame() const {
  return kindr::Position3D(estPositionWorldToBaseInWorld_);
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
kindr::LocalAngularVelocityPD
AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getAngularVelocityBaseInBaseFrame() const {
  return kindr::LocalAngularVelocityPD(estAngVelWorldToBaseInBase_);
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
Eigen::Matrix<double, 3, 1> AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getImuLinearAccelerationBias()
    const {
  return imuLinearAccBias_;
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
Eigen::Matrix<double, 3, 1> AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getImuAngularVelocityBias() const {
  return imuAngularVelBias_;
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
typename AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::PoseCovarianceMatrix
AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getEstPoseInOdomCovariance() const {
  PoseCovarianceMatrix output(PoseCovarianceMatrix::Identity());
  const auto positionIndex = odometryFilter_.GetPositionWorldToBaseInWorldIndex();
  const auto orientationIndex = odometryFilter_.GetOrientationBaseToWorldIndex();
  output.template topLeftCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(positionIndex, positionIndex);
  output.template topRightCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(positionIndex, orientationIndex);
  output.template bottomLeftCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(orientationIndex, positionIndex);
  output.template bottomRightCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(orientationIndex, orientationIndex);
  return output;
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
typename AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::PoseCovarianceMatrix
AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getEstTwistInBaseCovariance() const {
  PoseCovarianceMatrix twistStateCovariance(PoseCovarianceMatrix::Identity());
  const auto linVelIndex = odometryFilter_.GetLinearVelocityWorldToImuInBaseIndex();
  const auto angVelIndex = odometryFilter_.GetAngularVelocityWorldToBaseInBaseIndex();
  twistStateCovariance.template topLeftCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(linVelIndex, linVelIndex);
  twistStateCovariance.template topRightCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(linVelIndex, angVelIndex);
  twistStateCovariance.template bottomLeftCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(angVelIndex, linVelIndex);
  twistStateCovariance.template bottomRightCorner<3, 3>() = odometryStateCovariance_.block<3, 3>(angVelIndex, angVelIndex);

  // Twist filter state is (linVelworldToImuInBase, angVelWorldToBaseInBase), but we want the covariance of (linVelworldToBaseInBase, angVelWorldToBaseInBase)
  PoseCovarianceMatrix jacobian(PoseCovarianceMatrix::Identity());
  jacobian.template topRightCorner<3, 3>() = tsif::SSM(baseToImuPosition_);
  return jacobian*twistStateCovariance*jacobian.transpose();
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
any_measurements::Time AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getLastImuStamp() const {
  return prevMeasImuStamp_;
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
kindr::RotationQuaternionPD AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getOrientationBaseToWorld() const {
  return kindr::RotationQuaternionPD(estOrientationBaseToWorld_);
}

template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
kindr::Velocity3D AnymalFilterTsif<ConcreteDescription_, RobotState_, OdometryFilter_>::getLinearVelocityBaseInWorldFrame() const {
  const tsif::Mat3 estBaseToWorldRotationMatrix = estOrientationBaseToWorld_.toRotationMatrix();
  const tsif::Vec3 estLinVelWorldToBaseInWorld =
      estBaseToWorldRotationMatrix *
      (estLinVelWorldToImuInBase_ - tsif::SSM(estAngVelWorldToBaseInBase_) * baseToImuPosition_);
  return kindr::Velocity3D(estLinVelWorldToBaseInWorld);
}

}  // namespace anymal_state_estimator_tsif
