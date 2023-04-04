/*
 * WholeBodyController.cpp
 *
 *  Created on: Jun 5, 2015
 *      Author: Dario Bellicoso
 */

// loco_anymal
#include "whole_body_control_romo/motion_control/WholeBodyController.hpp"

// kindr
#include <kindr/Core>

namespace whole_body_control_romo {

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::addOptimizationTaskDistributeTangentialContactForces(int priority) {
  const auto numContacts = supportJacobian_.getNumberOfContacts();

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*numContacts, solutionSpaceDimension_);
  A.rightCols(3*numContacts).setIdentity();

  // Get the orientation from the control frame to the frame in which the forces are defined.
  const auto& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();
  const auto& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  const loco::RotationQuaternion& orientationControlToForceFrame =
      (supportJacobian_.getForceFrame() == CoordinateFrameEnum::WORLD)
    ? orientationWorldToBase.inverted()*orientationControlToBase
    : orientationControlToBase;

  const Eigen::MatrixXd headingDirectionInForceFrameTranspose = orientationControlToForceFrame.rotate(loco::Vector::UnitX()).toImplementation().transpose();
  const Eigen::MatrixXd lateralDirectionInForceFrameTranspose = orientationControlToForceFrame.rotate(loco::Vector::UnitY()).toImplementation().transpose();

  constexpr auto numConstraintsPerContact = 2;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(numConstraintsPerContact*numContacts, 3*numContacts);
  for (int k=0; k<numContacts; ++k) {
    B.block<1,3>(numConstraintsPerContact*k,   3*k) = headingDirectionInForceFrameTranspose;
    B.block<1,3>(numConstraintsPerContact*k+1, 3*k) = lateralDirectionInForceFrameTranspose;
  }

  return hierarchicalOptimization_->addOptimizationProblem(
      "contact_force_minimization", priority, B*A,
      Eigen::VectorXd::Zero(numConstraintsPerContact*numContacts),
      Eigen::MatrixXd(), Eigen::VectorXd(), hopt::ConstraintType::Equality);
}


template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::addOptimizationTaskDistributeTorques(int priority) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(RD::getJointsDimension(),solutionSpaceDimension_);
  A.block<RD::getJointsDimension(), RD::getJointsDimension()>(0, RD::getNumDof()).setIdentity();

  return hierarchicalOptimization_->addOptimizationProblem(
      "distribute_torques", priority,
      std::move(A), Eigen::VectorXd::Zero(RD::getJointsDimension()),
      Eigen::MatrixXd(), Eigen::VectorXd(),
      hopt::ConstraintType::Equality);
}

} /* namespace loco_anymal */
