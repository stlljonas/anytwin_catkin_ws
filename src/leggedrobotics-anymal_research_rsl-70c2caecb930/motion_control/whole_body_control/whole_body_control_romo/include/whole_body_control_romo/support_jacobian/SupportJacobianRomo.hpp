/*
 * SupportJacobianRomo.hpp
 *
 *  Created on: Jan 27, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// Eigen
#include <Eigen/Core>

// support jacobian interface
#include <whole_body_control/SupportJacobianQrDecomposition.hpp>

// robot model
#include <romo/RobotModel.hpp>

// std utils
#include <std_utils/timers/ChronoTimer.hpp>
#include <std_utils/containers/EnumArray.hpp>
#include <std_utils/containers/CompileTimeMap.hpp>

namespace whole_body_control_romo {

CONSECUTIVE_ENUM(ContactStateEnum, ContactOpen, ContactClosed3Dof, ContactClosed6Dof)

template <ContactStateEnum contactState, unsigned int uint>
using contactStateToUIntKV = std_utils::KeyValuePair<ContactStateEnum, unsigned int, contactState, uint>;
using mapContactStateToNumDofContact =
std_utils::CompileTimeMap<ContactStateEnum , unsigned int,
                          contactStateToUIntKV<ContactStateEnum::ContactOpen,       0u>,
                          contactStateToUIntKV<ContactStateEnum::ContactClosed3Dof, 3u>,
                          contactStateToUIntKV<ContactStateEnum::ContactClosed6Dof, 6u>>;

template<typename ConcreteDescription_, typename RobotState_>
class SupportJacobianRomo : public wbc::SupportJacobianQrDecomposition {
 public:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;
  using ContactEnum = typename RD::ContactEnum;
  using BranchEnum = typename RD::BranchEnum;
  using LimbEnum = typename RD::LimbEnum;
  using BodyEnum = typename RD::BodyEnum;
  using BodyNodeEnum = typename RD::BodyNodeEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;

  using ContactFlags = std_utils::EnumArray<LimbEnum, ContactStateEnum>;
  using StartIndexes = std_utils::EnumArray<LimbEnum, int>;

  explicit SupportJacobianRomo(const RobotModel& model,
                               CoordinateFrameEnum forceFrame = CoordinateFrameEnum::WORLD,
                               bool computeOrthogonalDecomposition = true);
  ~SupportJacobianRomo() override = default;

  SupportJacobianRomo(SupportJacobianRomo&&) = delete;
  SupportJacobianRomo& operator=(SupportJacobianRomo&&) = delete;

  SupportJacobianRomo(const SupportJacobianRomo&) = default;
  SupportJacobianRomo& operator=(const SupportJacobianRomo&) = default;

  virtual bool setContactFlags(const ContactFlags& contactFlags, bool updateSupportState);

  const Eigen::MatrixXd& getSupportJacobianInForceFrame() const override final;
  const Eigen::MatrixXd& getSupportJacobianTransposeInForceFrame() const override final;
  const Eigen::MatrixXd& getSupportJacobianTimeDerivativeInForceFrame() const override final;

  const Eigen::MatrixXd& getP() const override final;
  const Eigen::MatrixXd& getQ() const override final;
  const Eigen::MatrixXd& getR() const override final;
  const Eigen::MatrixXd& getInvR() const override final;
  const Eigen::MatrixXd& getQu() const override final;
  const Eigen::MatrixXd& getQc() const override final;
  const Eigen::MatrixXd& getSu() const override final;
  const Eigen::MatrixXd& getSc() const override final;

  //! Returns the number of contact locations.
  virtual int getNumberOfContacts() const;

  //! Returns the number of constraints that a contact can provide.
  // Open: 0
  // Point contact: 3
  // Extended contact: 6
  unsigned int getNumberOfContactConstraintsFromContactEnum(ContactStateEnum contactStateEnum) const;

  //! Returns the total number of constraints that the support Jacobian will provide.
  unsigned int getNumberOfTotalContactConstraints() const;

  //! Returns the index in the support Jacobian belonging to the first row of the block belonging to the specified limb. Returns -1 when ContactOpen.
  int getStartIndexInSupportJacobian(LimbEnum limbEnum) const;

  //! The frame in which the reaction forces are expressed.
  CoordinateFrameEnum getForceFrame() const;

 protected:
  //! This method cycles through the limbs which are in contact and stacks the Jacobians of the contact locations into the support Jacobian.
  bool updateSupport() override final;

  //! A reference to the robot model.
  const RobotModel& model_;

  //! Vector of contact flags.
  ContactFlags contactFlags_;

  //! The support Jacobian in force fame.
  Eigen::MatrixXd supportJacobianInForceFrame_;

  //! The transpose of the support Jacobian in force fame.
  Eigen::MatrixXd supportJacobianTransposeInForceFrame_;

  //! The time derivative of the support Jacobian in force fame.
  Eigen::MatrixXd supportJacobianTimeDerivativeInForceFrame_;

  //! The starting indexes of each limb in the support Jacobian.
  StartIndexes startIndexesForLimbsInSupportJacobian_;

  //! Selection matrices.
  Eigen::MatrixXd selectionMatrixActuators_;
  Eigen::MatrixXd selectionMatrixConstrained_;
  Eigen::MatrixXd selectionMatrixUnconstrained_;

  //! QR factorization of the support Jacobian.
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd invR_;
  Eigen::MatrixXd Qu_;
  Eigen::MatrixXd Qc_;

  //! Projection to the nullspace of Js'.
  Eigen::MatrixXd Pf_;

  //! A std::chrono based timer.
  std_utils::HighResolutionClockTimer timer_;

  //! When true, the QR decomposition of the support Jacobian will be computed.
  bool computeOrthogonalDecomposition_;

  //! The frame in which the reaction forces x_F are expressed.
  //! This will be the same frame 'x' in which the support Jacobian Js
  //! will be computed, i.e.:
  //!     x_Js' * x_F
  CoordinateFrameEnum forceFrame_;
};

} /* namespace whole_body_control_romo */

#include "whole_body_control_romo/support_jacobian/SupportJacobianRomo.tpp"
