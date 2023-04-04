/*
 * FrictionConstraint.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#include "loco/contact_force_distribution/constraints/FrictionConstraint.hpp"

namespace loco {

// Define nDirections declared in header
constexpr unsigned int FrictionConstraint::nDirections_;

FrictionConstraint::FrictionConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain)
    : ConstraintInterface("FrictionConstraint", wholeBody, terrain) {}

bool FrictionConstraint::updateConstraint(const LimbInfos& limbInfos) {
  /* For each tangent direction t, we want: -mu * n.f_i <= t.f_i <= mu * n.f_i
   * with n.f_i the normal component of contact force of leg i, t.f_i the tangential component).
   * This is equivalent to the two constraints: mu * n.f_i >= -t.f_i and mu * n.f_i >= t * f_i,
   * and equivalently -mu * n.f_i - t.f_i <=0 and -mu * n.f_i + t.f_i <= 0.
   * We have to define these constraints for both tangential directions (approximation of the
   * friction cone).
   */
  const unsigned int nConstraints = nDirections_ * limbInfos.getNumLimbsInForceDistribution();
  const RotationQuaternion& orientationWorldToBase = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase();
  const RotationQuaternion& orientationControlToBase =
      wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationControlToBase();

  // Resize jacobian
  globalIneqConJacobian_.resize(nConstraints, limbInfos.getNumLimbsInForceDistribution() * cartesian::forceVectorSize);

  ineqConMaxValues_.resize(nConstraints);
  ineqConMaxValues_.setConstant(std::numeric_limits<double>::max());

  ineqConMinValues_.resize(nConstraints);
  ineqConMinValues_.setZero();

  Eigen::MatrixXd frictionConstraint(nDirections_, limbInfos.getNumLimbsInForceDistribution() * cartesian::forceVectorSize);

  unsigned int rowOffset = 0;

  for (const auto& limb : wholeBody_.getLimbs()) {
    if (limbInfos[limb].isPartOfForceDistribution_) {
      const Position positionWorldToFootInWorldFrame =
          limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      Vector footContactNormalInWorldFrame;
      terrain_.getNormal(positionWorldToFootInWorldFrame, footContactNormalInWorldFrame);
      Vector footContactNormalInBaseFrame = orientationWorldToBase.rotate(footContactNormalInWorldFrame);
      const Eigen::Vector3d normalDirection = footContactNormalInBaseFrame.toImplementation();

      // for logging
      frictionCones_[limb->getLimbUInt()].normalDirectionInWorldFrame_ = loco::Vector(footContactNormalInWorldFrame);

      // The choose the first tangential to lie in the XZ-plane of the base frame.
      // This is the same as the requirement as
      // 1) firstTangential perpendicular to normalDirection,
      // 2) firstTangential perpendicular to normal of XZ-plane of the base frame,
      // 3) firstTangential has unit norm.
      //      Vector3d firstTangential = normalDirection.cross(Vector3d::UnitY()).normalized();

      const Eigen::Vector3d vectorY = Eigen::Vector3d::UnitY();
      Eigen::Vector3d firstTangentialInBaseFrame = orientationControlToBase.rotate(vectorY);
      Eigen::Vector3d firstTangential = normalDirection.cross(firstTangentialInBaseFrame).normalized();

      // logging
      frictionCones_[limb->getLimbUInt()].primaryDirectionInWorldFrame_ =
          loco::Vector(orientationWorldToBase.inverseRotate(firstTangential));

      // The second tangential is perpendicular to the normal and the first tangential.
      Eigen::Vector3d secondTangential = normalDirection.cross(firstTangential).normalized();

      // logging
      frictionCones_[limb->getLimbUInt()].secondaryDirectionInWorldFrame_ =
          loco::Vector(orientationWorldToBase.inverseRotate(secondTangential));

      // get friction coefficient at foot position
      terrain_.getFrictionCoefficientForFoot(limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame(),
                                             frictionCones_[limb->getLimbUInt()].frictionCoefficient_);

      double modulatedFriction = frictionCones_[limb->getLimbUInt()].frictionCoefficient_ * limb->getFrictionModulation();

      // @fixme Hack to test contact invariance (Christian)
      if (limb->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant) {
        modulatedFriction = 0.02;
      }

      // Set zero
      frictionConstraint.setZero();

      // First tangential, positive
      frictionConstraint.block<1, cartesian::forceVectorSize>(0, limbInfos[limb].limbStartIndexInSolutionVector_) =
          modulatedFriction * normalDirection.transpose() + firstTangential.transpose();
      // First tangential, negative
      frictionConstraint.block<1, cartesian::forceVectorSize>(1, limbInfos[limb].limbStartIndexInSolutionVector_) =
          modulatedFriction * normalDirection.transpose() - firstTangential.transpose();
      // Second tangential, positive
      frictionConstraint.block<1, cartesian::forceVectorSize>(2, limbInfos[limb].limbStartIndexInSolutionVector_) =
          modulatedFriction * normalDirection.transpose() + secondTangential.transpose();
      // Second tangential, negative
      frictionConstraint.block<1, cartesian::forceVectorSize>(3, limbInfos[limb].limbStartIndexInSolutionVector_) =
          modulatedFriction * normalDirection.transpose() - secondTangential.transpose();

      // Set to jacobian
      globalIneqConJacobian_.middleRows(rowOffset, nDirections_) = frictionConstraint.sparseView();

      rowOffset += nDirections_;
    }
  }

  return true;
}

} /* namespace loco */
