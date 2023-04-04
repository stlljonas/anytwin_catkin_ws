#pragma once

#include <any_measurements/Imu.hpp>
#include <any_measurements/Time.hpp>
#include <any_node/Node.hpp>

#include <anymal_model/StateStatus.hpp>
#include <anymal_model/AnymalState.hpp>
#include <romo/RobotModel.hpp>

// kindr
#include <kindr/Core>

#include <any_measurements/PoseWithCovariance.hpp>
#include <any_measurements_ros/ConvertRosMessages.hpp>

#include <anymal_state_estimator/anymal_state_estimator_utils/Contact.hpp>

#include <any_node/Param.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

namespace anymal_state_estimator {

/**
 * @brief      Generic interface to be used for filters performing state
 *             estimation inside AnymalStateEstimator
 *
 * @tparam     ConcreteDescription_  romo::ConcreteDescription
 * @tparam     RobotState_           romo::RobotState
 */
template <typename ConcreteDescription_, typename RobotState_>
class AnymalFilter {
 public:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;
  using CT = typename ConcreteDescription_::ConcreteTopology;
  using JointEnum = typename CT::JointEnum;
  using ContactEnum = typename CT::ContactEnum;
  using StateStatus = anymal_model::StateStatus;
  using PoseCovarianceMatrix = Eigen::Matrix<double, RD::getNumSpatialDof(), RD::getNumSpatialDof()>;
  using JointState = std_utils::EnumArray<JointEnum, any_measurements::ExtendedJointState>;

  template <typename ValueType_>
  using ContactEnumContainer = std_utils::EnumArray<ContactEnum, ValueType_>;

  AnymalFilter() = delete;

  explicit AnymalFilter(any_node::Node::NodeHandlePtr nh) : nh_(std::move(nh)) {}

  virtual ~AnymalFilter() = default;
  virtual bool initFilter(const kindr::HomTransformQuatD& poseBaseToOdom) = 0;
  virtual bool resetFilter(const kindr::HomTransformQuatD& poseBaseToOdom) = 0;

  virtual kindr::RotationQuaternionPD getOrientationBaseToWorld() const = 0;
  virtual kindr::Velocity3D getLinearVelocityBaseInWorldFrame() const = 0;
  virtual kindr::LocalAngularVelocityPD getAngularVelocityBaseInBaseFrame() const = 0;
  virtual kindr::Position3D getPositionWorldToBaseInWorldFrame() const = 0;
  virtual Eigen::Matrix<double, 3, 1> getImuLinearAccelerationBias() const = 0;
  virtual Eigen::Matrix<double, 3, 1> getImuAngularVelocityBias() const = 0;

  virtual PoseCovarianceMatrix getEstPoseInOdomCovariance() const = 0;
  virtual PoseCovarianceMatrix getEstTwistInBaseCovariance() const = 0;

  virtual any_measurements::Time getLastImuStamp() const = 0;
  virtual StateStatus getStatus() const = 0;

  virtual void processImuReadings(const any_measurements::Imu& imu) = 0;
  virtual void processKinematics(const JointState& joints, const ContactEnumContainer<Contact>& contacts,
                                 const bool fakeKinematicsUpdateActive) = 0;
  virtual void updateFilter() = 0;

  virtual void addVariablesToLog() { /* do nothing */ }
  virtual void setModelPtr(std::shared_ptr<RobotModel> modelPtr) { /* do nothing */ }
  virtual void updateFullContactFlag(const bool hasFullContact) { initialFullContactSet_ = hasFullContact; }

  any_measurements::Time getLastEstimatedStateStamp() const { return lastEstimatedStateStamp_; }

 protected:
  any_node::Node::NodeHandlePtr nh_;
  Eigen::Vector3d previousAngularVelocityMeasurement_ = Eigen::Vector3d::Zero();
  any_measurements::Time lastEstimatedStateStamp_;
  bool initialFullContactSet_{false};
};

}  // namespace anymal_state_estimator
