/*
 * BaseTaskStateRomo.tpp
 *
 *  Created on: July 15, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// wholebody_romo
#include "whole_body_control_romo/BaseTaskStateRomo.hpp"

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
BaseTaskStateRomo<ConcreteDescription_, RobotState_>::BaseTaskStateRomo(const RobotModel& model, const loco::WholeBody& wholeBody)
    : model_(model),
      wholeBody_(wholeBody),
      orientationErrorInControlFrame_(),
      angularVelocityErrorInControlFrame_(),
      positionErrorInControlFrame_(),
      linearVelocityErrorInControlFrame_(),
      jacobianSpatialWorldToTargetInWorldFrame_(Eigen::MatrixXd::Zero(6, RD::getNumDof())),
      jacobianSpatialDerivativeWorldToTargetInWorldFrame_(Eigen::MatrixXd::Zero(6, RD::getNumDof())),
      jacobianTranslationWorldToTargetInControlFrame_(Eigen::MatrixXd::Zero(3, RD::getNumDof())),
      jacobianTranslationDerivativeWorldToTargetInControlFrame_(Eigen::MatrixXd::Zero(3, RD::getNumDof())),
      jacobianRotationWorldToTargetInControlFrame_(Eigen::MatrixXd::Zero(3, RD::getNumDof())),
      jacobianRotationDerivativeWorldToTargetInControlFrame_(Eigen::MatrixXd::Zero(3, RD::getNumDof())) {}

template <typename ConcreteDescription_, typename RobotState_>
bool BaseTaskStateRomo<ConcreteDescription_, RobotState_>::advance(double dt) {
  computeErrors();
  computeTargetJacobians();
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool BaseTaskStateRomo<ConcreteDescription_, RobotState_>::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(positionErrorInControlFrame_, "positionErrorTargetInControlFrame", "/whole_body_control/base_translation/", "m");
  signal_logger::add(linearVelocityErrorInControlFrame_, "linearVelocityErrorTargetInControlFrame", "/whole_body_control/base_translation/", "m/s");
  signal_logger::add(orientationErrorInControlFrame_, "orientationErrorTargetInControlFrame", "/whole_body_control/base_rotation/", "-");
  signal_logger::add(angularVelocityErrorInControlFrame_, "angularVelocityErrorTargetInControlFrame", "/whole_body_control/base_rotation/", "rad/s");
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool BaseTaskStateRomo<ConcreteDescription_, RobotState_>::computeErrors() {
  // Compute the control errors.
  const loco::RotationQuaternion& orientationControlToBase =
      wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationControlToBase();
  const loco::RotationQuaternion& orientationControlToDesiredBase = wholeBody_.getTorso().getDesiredState().getOrientationControlToBase();

  // Configuration errors.
  positionErrorInControlFrame_ = wholeBody_.getTorso().getDesiredState().getPositionErrorInControlFrame();
  orientationErrorInControlFrame_ = orientationControlToDesiredBase.inverted().boxMinus(orientationControlToBase.inverted());

  // Twist errors.
  linearVelocityErrorInControlFrame_ = wholeBody_.getTorso().getDesiredState().getLinearVelocityErrorInControlFrame();
  angularVelocityErrorInControlFrame_ =
      wholeBody_.getTorso().getDesiredState().getAngularVelocityBaseInControlFrame() -
      orientationControlToBase.inverseRotate(wholeBody_.getTorso().getMeasuredState().getAngularVelocityBaseInBaseFrame());
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool BaseTaskStateRomo<ConcreteDescription_, RobotState_>::computeTargetJacobians() {
  // Reset jacobians
  jacobianSpatialWorldToTargetInWorldFrame_.setZero();
  jacobianSpatialDerivativeWorldToTargetInWorldFrame_.setZero();

  switch (wholeBody_.getTorso().getDesiredState().getTargetPoint()) {
    case (loco::TorsoStateDesired::TargetPoint::BASE): {
      // The control target pose is a frame attached to the origin of the base frame and aligned to it.
      model_.getJacobianSpatialWorldToBody(jacobianSpatialWorldToTargetInWorldFrame_, RD::BranchEnum::BASE, RD::BodyNodeEnum::BASE,
                                           RD::CoordinateFrameEnum::WORLD);
      model_.getJacobianSpatialTimeDerivativeWorldToBody(jacobianSpatialDerivativeWorldToTargetInWorldFrame_,
                                                         jacobianSpatialWorldToTargetInWorldFrame_, RD::BranchEnum::BASE,
                                                         RD::BodyNodeEnum::BASE, RD::CoordinateFrameEnum::WORLD);
    } break;

    case (loco::TorsoStateDesired::TargetPoint::TORSOCOM): {
      // The target pose is attached to the torso center of mass and aligned with the base frame.
      model_.getJacobianSpatialWorldToPointOnBody(
          jacobianSpatialWorldToTargetInWorldFrame_,
          wholeBody_.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame().toImplementation(), RD::BranchEnum::BASE,
          RD::BodyNodeEnum::BASE, RD::CoordinateFrameEnum::WORLD);

      // Get the timederivative of the spatial jacobian.
      model_.getJacobianSpatialTimeDerivativeWorldToPointOnBody(
          jacobianSpatialDerivativeWorldToTargetInWorldFrame_, jacobianSpatialWorldToTargetInWorldFrame_,
          wholeBody_.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame().toImplementation(), RD::BranchEnum::BASE,
          RD::BodyNodeEnum::BASE, RD::CoordinateFrameEnum::WORLD);
    } break;

    case (loco::TorsoStateDesired::TargetPoint::WBCOM): {
      // The target pose is attached to the whole body center of mass and aligned with the base frame.
      Eigen::MatrixXd jacobianTranslationWorldToComInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
      model_.getJacobianTranslationWorldToCom(jacobianTranslationWorldToComInWorldFrame, RD::CoordinateFrameEnum::WORLD);

      Eigen::MatrixXd jacobianRotationWorldToBaseInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
      model_.getJacobianRotationWorldToBody(jacobianRotationWorldToBaseInWorldFrame, RD::BranchEnum::BASE, RD::BodyNodeEnum::BASE,
                                            RD::CoordinateFrameEnum::WORLD);
      jacobianSpatialWorldToTargetInWorldFrame_ << jacobianRotationWorldToBaseInWorldFrame, jacobianTranslationWorldToComInWorldFrame;

      // Get the time derivative of the spatial jacobian.
      // FIXME
      model_.getJacobianSpatialTimeDerivativeWorldToPointOnBody(
          jacobianSpatialDerivativeWorldToTargetInWorldFrame_, jacobianSpatialWorldToTargetInWorldFrame_,
          wholeBody_.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame().toImplementation(), RD::BranchEnum::BASE,
          RD::BodyNodeEnum::BASE, RD::CoordinateFrameEnum::WORLD);
    } break;

    case (loco::TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
      /* The target pose is attached to the a point defined as:
       *   x-y: whole body center of mass
       *   z: torso center
       * The target pose is aligned with the base frame.
       */
      // The target pose is attached to the whole body center of mass and aligned with the base frame.
      Eigen::MatrixXd jacobianTranslationWorldToComInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
      model_.getJacobianTranslationWorldToCom(jacobianTranslationWorldToComInWorldFrame, RD::CoordinateFrameEnum::WORLD);

      Eigen::MatrixXd jacobianTranslationWorldToBaseInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
      model_.getJacobianTranslationWorldToBody(
          jacobianTranslationWorldToBaseInWorldFrame,
          RD::BranchEnum::BASE, RD::BodyNodeEnum::BASE, RD::CoordinateFrameEnum::WORLD);

      Eigen::MatrixXd jacobianRotationWorldToBaseInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
      model_.getJacobianRotationWorldToBody(jacobianRotationWorldToBaseInWorldFrame, RD::BranchEnum::BASE, RD::BodyNodeEnum::BASE,
                                            RD::CoordinateFrameEnum::WORLD);

      jacobianSpatialWorldToTargetInWorldFrame_ <<
          jacobianRotationWorldToBaseInWorldFrame,
          jacobianTranslationWorldToComInWorldFrame.topRows<2>(),
          jacobianTranslationWorldToBaseInWorldFrame.bottomRows<1>();

      // Get the timederivative of the spatial jacobian.
      model_.getJacobianSpatialTimeDerivativeWorldToPointOnBody(
          jacobianSpatialDerivativeWorldToTargetInWorldFrame_, jacobianSpatialWorldToTargetInWorldFrame_,
          wholeBody_.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame().toImplementation(), RD::BranchEnum::BASE,
          RD::BodyNodeEnum::BASE, RD::CoordinateFrameEnum::WORLD);
    } break;
    default: {
      MELO_WARN_STREAM("[WholeBodyController::computeTargetJacobians] Could not compute target point jacobians!");
      return false;
    }
  }
  // Fill helper jacobians
  RotationMatrix orientationWorldToControl =
      RotationMatrix(wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl());
  jacobianTranslationWorldToTargetInControlFrame_ =
      orientationWorldToControl.matrix() * jacobianSpatialWorldToTargetInWorldFrame_.bottomRows<3>();
  jacobianRotationWorldToTargetInControlFrame_ =
      orientationWorldToControl.matrix() * jacobianSpatialWorldToTargetInWorldFrame_.topRows<3>();
  jacobianTranslationDerivativeWorldToTargetInControlFrame_ =
      orientationWorldToControl.matrix() * jacobianSpatialDerivativeWorldToTargetInWorldFrame_.bottomRows<3>();
  jacobianRotationDerivativeWorldToTargetInControlFrame_ =
      orientationWorldToControl.matrix() * jacobianSpatialDerivativeWorldToTargetInWorldFrame_.topRows<3>();
  return true;
}

}  // namespace whole_body_control_romo
