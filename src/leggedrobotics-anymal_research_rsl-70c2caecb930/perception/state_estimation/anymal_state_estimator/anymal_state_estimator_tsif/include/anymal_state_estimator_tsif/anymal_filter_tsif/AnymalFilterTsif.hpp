#pragma once

#include <anymal_state_estimator/anymal_filter/AnymalFilter.hpp>
#include <anymal_state_estimator_tsif/tsif/ImuKinAnymalOdometryTsif.hpp>

#include <anymal_state_estimator/anymal_state_estimator_utils/ImuInitializer.hpp>

namespace anymal_state_estimator_tsif {

using anymal_state_estimator::Contact;
using anymal_state_estimator::ImuInitializer;

/**
 * @brief      The two state information filter implementation of AnymalFilter
 *
 * @tparam     ConcreteDescription_  romo::ConcreteDescription
 * @tparam     RobotState_           romo::RobotState
 * @tparam     OdometryFilter_       tsif::AnymalOdometryTsif
 */
template <typename ConcreteDescription_, typename RobotState_, typename OdometryFilter_>
class AnymalFilterTsif : public anymal_state_estimator::AnymalFilter<ConcreteDescription_, RobotState_> {

  static_assert(tsif::is_anymal_odometry_tsif_compatible_t<ConcreteDescription_, RobotState_, OdometryFilter_>::value,
              "[AnymalFilterTsif] OdometryFilter_ for AnymalFilterTsif must derive from tsif::AnymalOdometryTsif with matching romo::ConcreteDescription and romo::RobotState");

 public:
  using Base = anymal_state_estimator::AnymalFilter<ConcreteDescription_, RobotState_>;
  using typename Base::CT;
  using typename Base::ContactEnum;
  using typename Base::JointState;
  using typename Base::PoseCovarianceMatrix;
  using typename Base::RD;
  using typename Base::RobotModel;
  using typename Base::StateStatus;

  using BodyEnum = typename CT::BodyEnum;

  template <typename ValueType_>
  using ContactEnumContainer = std_utils::EnumArray<ContactEnum, ValueType_>;

  AnymalFilterTsif(any_node::Node::NodeHandlePtr nh);

  ~AnymalFilterTsif() override = default;

  bool initFilter(const kindr::HomTransformQuatD& poseBaseToOdom) override;
  bool imuLinkAvailable(const std::string& imu_link);
  bool resetFilter(const kindr::HomTransformQuatD& poseBaseToOdom) override;

  void processImuReadings(const any_measurements::Imu& imu) override;
  void processKinematics(const JointState& joints, const ContactEnumContainer<Contact>& contacts,
                         const bool fakeKinematicsUpdateActive) override;

  void updateFilter() override;
  void addVariablesToLog() override;
  StateStatus getStatus() const override;

  kindr::RotationQuaternionPD getOrientationBaseToWorld() const override;
  kindr::Position3D getPositionWorldToBaseInWorldFrame() const override;
  kindr::Velocity3D getLinearVelocityBaseInWorldFrame() const override;
  kindr::LocalAngularVelocityPD getAngularVelocityBaseInBaseFrame() const override;
  tsif::Vec3 getAngularVelocityBaseInBaseFrameTsif() const;

  Eigen::Matrix<double, 3, 1> getImuLinearAccelerationBias() const override;
  Eigen::Matrix<double, 3, 1> getImuAngularVelocityBias() const override;

  PoseCovarianceMatrix getEstPoseInOdomCovariance() const override;
  PoseCovarianceMatrix getEstTwistInBaseCovariance() const override;

  any_measurements::Time getLastImuStamp() const override;
  void setModelPtr(std::shared_ptr<RobotModel> modelPtr) override { robotModelPtr_ = modelPtr; }

 protected:
  virtual void readParameters();
  virtual void processJointStates(const JointState& joints, const bool fakeKinematicsUpdateActive);
  virtual void processContacts(const ContactEnumContainer<Contact>& contacts, const bool fakeKinematicsUpdateActive);
  void setOutput();

  std::shared_ptr<RobotModel> robotModelPtr_{nullptr};

  // two state information filter for anymal odometry and filter definitions
  OdometryFilter_ odometryFilter_;

  // to filter the imu measurements
  tsif::Vec3 previousLinearAccelerationMeasurement_;

  // contact information needed across methods
  ContactEnumContainer<tsif::Vec3> measuredFootholds_;

  // imu offset parameters to transform measurements and estimates
  tsif::Mat3 imuToBaseRotationMatrix_;
  tsif::Vec3 baseToImuPosition_;

  // foot center to contact point vector in odom to adjust the measurements
  tsif::Vec3 positionFootToContactInWorld_;

  // number of contacts necessary to switch on imu bias estimation
  unsigned int gyroBiasEstimationContactCountThreshold_;
  unsigned int accelerometerBiasEstimationContactCountThreshold_;

  // various flags representing the current state of the estimator
  bool initialActuatorReadingsSet_;
  bool imuInitializationFinished_;
  bool odometryFilterReadyToUpdate_;
  bool setOutputFromExtrapolation_;

  // initializer to determine gyro bias and roll/pitch from imu measurements
  ImuInitializer imuInitializer_;

  // tsif-specific quantities to log
  tsif::Vec3 imuLinearAccelerationBiasCovarianceDiagonal_{tsif::Vec3::Zero()};
  tsif::Vec3 imuAngularVelocityBiasCovarianceDiagonal_{tsif::Vec3::Zero()};

  // tsif output
  tsif::MatX odometryStateCovariance_{tsif::ImuKinAnymalOdometryTsif::State::Dim(), tsif::ImuKinAnymalOdometryTsif::State::Dim()};

  tsif::Vec3 estPositionWorldToBaseInWorld_{tsif::Vec3::Zero()};
  tsif::Quat estOrientationBaseToWorld_{tsif::Quat::Identity()};
  tsif::Vec3 estLinVelWorldToImuInBase_{tsif::Vec3::Zero()};
  tsif::Vec3 estAngVelWorldToBaseInBase_{tsif::Vec3::Zero()};
  tsif::Vec3 imuLinearAccBias_{tsif::Vec3::Zero()};
  tsif::Vec3 imuAngularVelBias_{tsif::Vec3::Zero()};

  any_measurements::Time prevMeasImuStamp_;
};

}  // namespace anymal_state_estimator_tsif

// gets compiled by src/AnymalFilterTsifDefault.cpp
// #include <anymal_state_estimator_tsif/anymal_filter_tsif/AnymalFilterTsif.tpp>