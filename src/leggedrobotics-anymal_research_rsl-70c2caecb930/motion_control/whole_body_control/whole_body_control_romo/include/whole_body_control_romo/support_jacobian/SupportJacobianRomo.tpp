/*
 * SupportJacobianRomo.cpp
 *
 *  Created on: Jan 27, 2016
 *      Author: Dario Bellicoso
 */

// wbc
#include "whole_body_control_romo/support_jacobian/SupportJacobianRomo.hpp"

// eigen
#include <Eigen/QR>

namespace whole_body_control_romo {

template<typename ConcreteDescription_, typename RobotState_>
SupportJacobianRomo<ConcreteDescription_, RobotState_>::SupportJacobianRomo(
  const RobotModel& model,
  CoordinateFrameEnum forceFrame,
  bool computeOrthogonalDecomposition) :
    SupportJacobianQrDecomposition(),
    model_(model),
    contactFlags_(),
    supportJacobianInForceFrame_(),
    supportJacobianTransposeInForceFrame_(),
    supportJacobianTimeDerivativeInForceFrame_(),
    startIndexesForLimbsInSupportJacobian_(),
    timer_("[SupportJacobianRomo]"),
    computeOrthogonalDecomposition_(computeOrthogonalDecomposition),
    forceFrame_(forceFrame)
{
  contactFlags_.fill(ContactStateEnum::ContactOpen);
  startIndexesForLimbsInSupportJacobian_.fill(-1);

  selectionMatrixActuators_ = Eigen::MatrixXd::Zero(RD::getJointsDimension(), RD::getNumDof());
  selectionMatrixActuators_.bottomRightCorner(RD::getJointsDimension(), RD::getJointsDimension()).setIdentity();
}

template<typename ConcreteDescription_, typename RobotState_>
bool SupportJacobianRomo<ConcreteDescription_, RobotState_>::setContactFlags(
    const ContactFlags& contactFlags, bool updateSupportState) {
  contactFlags_ = contactFlags;

  if (updateSupportState) {
    return updateSupport();
  }

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool SupportJacobianRomo<ConcreteDescription_, RobotState_>::updateSupport() {
  // Reset the size of the support Jacobian and its time derivative.
  const auto numConstraints = getNumberOfTotalContactConstraints();
  supportJacobianInForceFrame_ = Eigen::MatrixXd(numConstraints, RD::getNumDof());
  supportJacobianTimeDerivativeInForceFrame_ = Eigen::MatrixXd(numConstraints, RD::getNumDof());

  unsigned int limbStartIndexInSupportJacobian = 0;
  for (const auto contactKey : RD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto branchEnum = RD::template mapEnums<BranchEnum>(contactEnum);
    const auto limbEnum = RD::template mapEnums<LimbEnum>(branchEnum);
    const auto contactStateEnum = contactFlags_[limbEnum];

    if (contactStateEnum != ContactStateEnum::ContactOpen) {
      const auto endBodyNodeEnum = RD::template mapEnums<BodyNodeEnum>(RD::getBranchEndBody(branchEnum));
      const auto positionEndBodyToContactPointInEndBodyFrame =
          model_.getContactContainer().at(
              contactEnum)->getPositionMovableParentToContactInMovableParentFrame().toImplementation();

      // Compute the constraint Jacobian for the contact.
      Eigen::MatrixXd jacobianSpatialWorldToBodyInForceFrame = Eigen::MatrixXd::Zero(6, RD::getNumDof());
      model_.getJacobianSpatialWorldToPointOnBody(
              jacobianSpatialWorldToBodyInForceFrame, positionEndBodyToContactPointInEndBodyFrame,
              branchEnum, endBodyNodeEnum, forceFrame_);

      Eigen::MatrixXd jacobianSpatialWorldToBodyInWorldFrame = Eigen::MatrixXd::Zero(6, RD::getNumDof());
      model_.getJacobianSpatialWorldToPointOnBody(
          jacobianSpatialWorldToBodyInWorldFrame, positionEndBodyToContactPointInEndBodyFrame,
          branchEnum, endBodyNodeEnum, CoordinateFrameEnum::WORLD);

      // Compute the time derivative of the constraint Jacobian for the contact.
      Eigen::MatrixXd jacobianSpatialTimeDerivativeWorldToEndEffectorInForceFrame = Eigen::MatrixXd::Zero(6, RD::getNumDof());
      model_.getJacobianSpatialTimeDerivativeWorldToPointOnBody(
          jacobianSpatialTimeDerivativeWorldToEndEffectorInForceFrame,
          jacobianSpatialWorldToBodyInWorldFrame, positionEndBodyToContactPointInEndBodyFrame,
          branchEnum, endBodyNodeEnum, forceFrame_);

      // Stack the individual constraint Jacobians in the support Jacobian (first rotational part (for 6DOF contact), then translational part).
      const auto numContactConstraints = getNumberOfContactConstraintsFromContactEnum(contactStateEnum);
      supportJacobianInForceFrame_.middleRows(limbStartIndexInSupportJacobian, numContactConstraints) =
          jacobianSpatialWorldToBodyInForceFrame.bottomRows(numContactConstraints);
      supportJacobianTimeDerivativeInForceFrame_.middleRows(limbStartIndexInSupportJacobian, numContactConstraints) =
          jacobianSpatialTimeDerivativeWorldToEndEffectorInForceFrame.bottomRows(numContactConstraints);

      startIndexesForLimbsInSupportJacobian_[limbEnum] = limbStartIndexInSupportJacobian;
      limbStartIndexInSupportJacobian += numContactConstraints;
    }
    else {
      startIndexesForLimbsInSupportJacobian_[limbEnum] = -1;
    }
  }

  supportJacobianTransposeInForceFrame_ = supportJacobianInForceFrame_.transpose();

  // If necessary, compute the QR decomposition of the transpose of the support Jacobian.
  if (computeOrthogonalDecomposition_) {
    selectionMatrixUnconstrained_.setZero(RD::getNumDof() - numConstraints, RD::getNumDof());
    selectionMatrixUnconstrained_.bottomRightCorner(RD::getNumDof() - numConstraints,
                                                    RD::getNumDof() - numConstraints).setIdentity();

    selectionMatrixConstrained_.setZero(numConstraints, RD::getNumDof());
    selectionMatrixConstrained_.topLeftCorner(numConstraints, numConstraints).setIdentity();

    Eigen::HouseholderQR<Eigen::MatrixXd> QR(supportJacobianInForceFrame_.transpose());
    Q_ = QR.householderQ();
    R_ = QR.matrixQR().triangularView<Eigen::Upper>().toDenseMatrix().topLeftCorner(numConstraints, numConstraints);
    invR_ = R_.inverse();
    Qu_ = Q_.rightCols(RD::getNumDof() - numConstraints);
    Qc_ = Q_.leftCols(numConstraints);
    Pf_ = Qu_.transpose();
  }

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
unsigned int SupportJacobianRomo<ConcreteDescription_, RobotState_>::getNumberOfTotalContactConstraints() const {
  auto contactsDof = 0u;
  for (const auto& contactFlag : contactFlags_) {
    contactsDof += getNumberOfContactConstraintsFromContactEnum(contactFlag);
  }
  return contactsDof;
}

template<typename ConcreteDescription_, typename RobotState_>
unsigned int SupportJacobianRomo<ConcreteDescription_, RobotState_>::getNumberOfContactConstraintsFromContactEnum(ContactStateEnum contactStateEnum) const {
  return whole_body_control_romo::mapContactStateToNumDofContact::at(contactStateEnum);
}

template<typename ConcreteDescription_, typename RobotState_>
int SupportJacobianRomo<ConcreteDescription_, RobotState_>::getStartIndexInSupportJacobian(LimbEnum limbEnum) const {
  return startIndexesForLimbsInSupportJacobian_[limbEnum];
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getSupportJacobianInForceFrame() const {
  return supportJacobianInForceFrame_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getSupportJacobianTransposeInForceFrame() const {
  return supportJacobianTransposeInForceFrame_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getSupportJacobianTimeDerivativeInForceFrame() const {
  return supportJacobianTimeDerivativeInForceFrame_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getP() const {
  return Pf_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getQ() const {
  return Q_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getR() const {
  return R_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getInvR() const {
  return invR_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getQu() const {
  return Qu_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getQc() const {
  return Qc_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getSu() const {
  return selectionMatrixUnconstrained_;
}

template<typename ConcreteDescription_, typename RobotState_>
const Eigen::MatrixXd& SupportJacobianRomo<ConcreteDescription_, RobotState_>::getSc() const {
  return selectionMatrixConstrained_;
}

template<typename ConcreteDescription_, typename RobotState_>
int SupportJacobianRomo<ConcreteDescription_, RobotState_>::getNumberOfContacts() const {
  return std::count_if(contactFlags_.begin(), contactFlags_.end(),
    [] (ContactStateEnum contactEnum) {
      return contactEnum != ContactStateEnum::ContactOpen;
    }
  );
}

template<typename ConcreteDescription_, typename RobotState_>
typename SupportJacobianRomo<ConcreteDescription_, RobotState_>::CoordinateFrameEnum SupportJacobianRomo<ConcreteDescription_, RobotState_>::getForceFrame() const {
  return forceFrame_;
}

} /* namespace whole_body_control_romo */
