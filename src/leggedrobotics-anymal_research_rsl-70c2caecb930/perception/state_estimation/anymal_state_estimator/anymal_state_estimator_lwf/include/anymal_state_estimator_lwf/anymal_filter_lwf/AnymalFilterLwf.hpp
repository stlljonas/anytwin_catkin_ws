#pragma once

#include <anymal_state_estimator/anymal_state_estimator_utils/Contact.hpp>

#include <anymal_state_estimator/anymal_filter/AnymalFilter.hpp>

// lwf models
#include <lightweight_filtering_models/CoordinateTransforms/OdometryToBaseOutputCT.hpp>
#include <lightweight_filtering_models/CoordinateTransforms/StandardKinOutputCF.hpp>
#include <lightweight_filtering_models/CoordinateTransforms/StandardPoseOutputCF.hpp>
#include <lightweight_filtering_models/CoordinateTransforms/TransformStandardPoseCF.hpp>
#include <lightweight_filtering_models/Filters/LegPosAndPoseFilter.hpp>
#include <lightweight_filtering_models/Filters/LegVelAndPoseFilter.hpp>

// state estimator lwf
#include <anymal_state_estimator_lwf/anymal_filter_lwf/KinematicsModelLwf.hpp>

// boost
#include <boost/thread.hpp>

namespace anymal_state_estimator_lwf {

/**
 * @brief      The lightweight filter implementation of AnymalFilter
 *
 * @tparam     ConcreteDescription_  romo::ConcreteDescription
 * @tparam     RobotState_           romo::RobotState
 * @tparam     LwfModelType_         kinematic model of the robot
 */
template <typename ConcreteDescription_, typename RobotState_, typename LwfModelType_>
class AnymalFilterLwf : public anymal_state_estimator::AnymalFilter<ConcreteDescription_, RobotState_> {
 public:
  using Base = anymal_state_estimator::AnymalFilter<ConcreteDescription_, RobotState_>;
  using typename Base::CT;
  using typename Base::ContactEnum;
  using typename Base::JointEnum;
  using typename Base::JointState;
  using typename Base::PoseCovarianceMatrix;
  using typename Base::RD;
  using typename Base::StateStatus;
  using Contact = anymal_state_estimator::Contact;

  template <typename ValueType_>
  using ContactEnumContainer = std_utils::EnumArray<ContactEnum, ValueType_>;

  AnymalFilterLwf() = delete;

  explicit AnymalFilterLwf(any_node::Node::NodeHandlePtr nh);

  ~AnymalFilterLwf() override;

  bool initFilter(const kindr::HomTransformQuatD& poseBaseToOdom) override;
  bool resetFilter(const kindr::HomTransformQuatD& poseBaseToOdom) override;
  void processImuReadings(const any_measurements::Imu& imu) override;
  void processKinematics(const JointState& joints, const ContactEnumContainer<Contact>& contacts,
                         const bool fakeKinematicsUpdateActive) override;

  void updateFilter() override;
  StateStatus getStatus() const override;

  kindr::RotationQuaternionPD getOrientationBaseToWorld() const override;
  kindr::Position3D getPositionWorldToBaseInWorldFrame() const override;

  kindr::LocalAngularVelocityPD getAngularVelocityBaseInBaseFrame() const override;

  Eigen::Matrix<double, 3, 1> getImuLinearAccelerationBias() const override;
  Eigen::Matrix<double, 3, 1> getImuAngularVelocityBias() const override;

  PoseCovarianceMatrix getEstPoseInOdomCovariance() const override;
  PoseCovarianceMatrix getEstTwistInBaseCovariance() const override;

  any_measurements::Time getLastImuStamp() const override;

  kindr::Velocity3D getLinearVelocityBaseInWorldFrame() const override;

 protected:
  kindr::RotationQuaternionPD getOrientationOdomToBase() const;
  kindr::Velocity3D getLinearVelocityInBaseFrame() const;

  using mtFilter = LWFM::LegPosAndPoseFilter<RD::getNumLegs(), LwfModelType_>;
  using mtPredictionMeas = typename mtFilter::mtPrediction::mtMeas;

  mtFilter filter_;

  using mtPoseMeas = typename std::tuple_element<0, decltype(filter_.mUpdates_)>::type::mtMeas;
  using mtKinUpdate = typename std::tuple_element<1, decltype(filter_.mUpdates_)>::type;
  using mtKinMeas = typename std::tuple_element<1, decltype(filter_.mUpdates_)>::type::mtMeas;
  using mtOutput = LWFM::StandardOutput;

  //! This needs to be protected by mutexFilter_.
  LwfModelType_* filterKinematicModelPtr_;
  LWFM::TransformStandardPoseCF transformStandardPoseCF_;

  //! This holds C_BG, G_r_GB, which can be stored in the transform T_MG = [inv(C_BG), G_r_GB]
  mtOutput alignedPoseOutput_;
  Eigen::MatrixXd alignedPoseOutputCov_;

  //! This holds C_BO, O_r_OB, which can be stored in the transform T_OB = [inv(C_BO), O_r_OB]
  mtOutput odometryOutput_;
  LWFM::OdometryToBaseOutputCT<mtKinUpdate> odometryToBaseOutputCT_;

  Eigen::MatrixXd odometryOutputCov_;
  any_measurements::Time prevMeasImuStamp_;

  bool isInitialized_{false};
};

}  // namespace anymal_state_estimator_lwf

// gets compiled by src/AnymalFilterLwfDefault.cpp
// #include <anymal_state_estimator_lwf/anymal_filter_lwf/AnymalFilterLwf.tpp>