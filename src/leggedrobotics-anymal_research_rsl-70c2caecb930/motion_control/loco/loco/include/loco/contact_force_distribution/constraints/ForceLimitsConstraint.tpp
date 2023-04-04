/*
 * ForceLimitsConstraint.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#include "loco/contact_force_distribution/constraints/ForceLimitsConstraint.hpp"
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace loco {

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
ForceLimitsConstraint<TranslationalCoordinates_>::ForceLimitsConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain,
                                                                        const ForceLimit& defaultForceLimit)
    : ConstraintInterface("ForceLimitsConstraint", wholeBody, terrain),
      defaultNormalGroundForceLimits_(defaultForceLimit),
      limbNormalGroundForceLimits_() {}

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
bool ForceLimitsConstraint<TranslationalCoordinates_>::updateConstraint(const LimbInfos& limbInfos) {
  /* We want each stance leg to have a minimal/maximal force in the normal direction to the ground:
   * n.f_i >= n.f_min, equivalent to  -n.f_min >= -n.f_i,  with n.f_i the normal component of contact force.
   * n.f_i <= n.f_max, equivalent to   n.f_max >= n.f_i,   with n.f_i the normal component of contact force.
   */

  const RotationQuaternion& orientationWorldToBase = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase();
  const unsigned int numLimbsInForceDistribution = limbInfos.getNumLimbsInForceDistribution();
  constexpr unsigned int forceVecSize = cartesian::getTranslationalCoordinatesCount(TranslationalCoordinates_);

  // Resize jacobian
  unsigned int rowOffset = 0;
  globalIneqConJacobian_.resize(numLimbsInForceDistribution, forceVecSize * numLimbsInForceDistribution);
  ineqConMaxValues_.resize(numLimbsInForceDistribution);
  ineqConMinValues_.resize(numLimbsInForceDistribution);
  Eigen::VectorXd forceConstraintRow(forceVecSize * numLimbsInForceDistribution);

  for (const auto& limb : wholeBody_.getLimbs()) {
    if (limbInfos[limb].isPartOfForceDistribution_) {
      const Position& positionWorldToFootInWorldFrame =
          limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      Vector footContactNormalInWorldFrame;
      terrain_.getNormal(positionWorldToFootInWorldFrame, footContactNormalInWorldFrame);
      const Vector footContactNormalInBaseFrame = orientationWorldToBase.rotate(footContactNormalInWorldFrame);

      forceConstraintRow.setZero();

      switch (TranslationalCoordinates_) {
        case cartesian::TranslationalCoordinates::XYZ:
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_ + 2) = footContactNormalInBaseFrame(2);
        case cartesian::TranslationalCoordinates::XY:
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_ + 1) = footContactNormalInBaseFrame(1);
        case cartesian::TranslationalCoordinates::X:
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_) = footContactNormalInBaseFrame(0);
          break;
        case cartesian::TranslationalCoordinates::YZ:
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_ + 1) = footContactNormalInBaseFrame(2);
        case cartesian::TranslationalCoordinates::Y:
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_) = footContactNormalInBaseFrame(1);
          break;
        case cartesian::TranslationalCoordinates::Z:
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_) = footContactNormalInBaseFrame(2);
          break;
        case cartesian::TranslationalCoordinates::XZ:
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_ + 1) = footContactNormalInBaseFrame(2);
          forceConstraintRow(limbInfos[limb].limbStartIndexInSolutionVector_) = footContactNormalInBaseFrame(0);
          break;
      }
      globalIneqConJacobian_.middleRows(rowOffset, 1) = (forceConstraintRow.transpose()).sparseView();

      // Limit with leg loading
      const ForceLimit& normalGroundForceLimit = getLimit(limb->getLimbUInt());
      double minGroundForce = normalGroundForceLimit.minForce_;
      double maxGroundForce = normalGroundForceLimit.maxForce_;

      if (normalGroundForceLimit.maxIncrement_ != std::numeric_limits<double>::max()) {
        // Normal force represents the force applied by the environment on the robot.
        const double measuredNormalForce =
            (footContactNormalInBaseFrame.toImplementation().transpose() *
             orientationWorldToBase.rotate(
                 limb->getEndEffector().getStateMeasured().getForceAtEndEffectorInWorldFrame().toImplementation()));
        minGroundForce = std::max(measuredNormalForce - normalGroundForceLimit.maxIncrement_, normalGroundForceLimit.minForce_);
        maxGroundForce = std::min(measuredNormalForce + normalGroundForceLimit.maxIncrement_, normalGroundForceLimit.maxForce_);
      }

      if (limbInfos[limb].isLoadConstraintActive_) {
        maxGroundForce = std::min(maxGroundForce, normalGroundForceLimit.maxForceLoadConstrained_ * limb->getLoadFactor());
      }

      // Test necessary if load constraint is active
      ineqConMinValues_(rowOffset) = std::min(minGroundForce, maxGroundForce);
      ineqConMaxValues_(rowOffset) = maxGroundForce;

      // Increment offset
      rowOffset += 1;
    }
  }

  return true;
}

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
bool ForceLimitsConstraint<TranslationalCoordinates_>::setToInterpolated(const ConstraintInterface& constraint1,
                                                                         const ConstraintInterface& constraint2, double t) {
  const auto& forceConstraint1 = static_cast<const ForceLimitsConstraint<TranslationalCoordinates_>&>(constraint1);
  const auto& forceConstraint2 = static_cast<const ForceLimitsConstraint<TranslationalCoordinates_>&>(constraint2);

  // Interpolate default force limits
  defaultNormalGroundForceLimits_.setToInterpolated(forceConstraint1.defaultNormalGroundForceLimits_,
                                                    forceConstraint2.defaultNormalGroundForceLimits_, t);

  // Merge the two maps
  limbNormalGroundForceLimits_ = forceConstraint1.limbNormalGroundForceLimits_;
  limbNormalGroundForceLimits_.insert(forceConstraint2.limbNormalGroundForceLimits_.begin(),
                                      forceConstraint2.limbNormalGroundForceLimits_.end());
  // Interpolate limb specific limits
  for (auto& gfl : limbNormalGroundForceLimits_) {
    gfl.second.setToInterpolated(forceConstraint1.getLimit(gfl.first), forceConstraint2.getLimit(gfl.first), t);
  }

  return true;
}

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
void ForceLimitsConstraint<TranslationalCoordinates_>::setDefaultNormalGroundForceLimits(const ForceLimit& defaultNormalGroundForceLimits) {
  defaultNormalGroundForceLimits_ = defaultNormalGroundForceLimits;
}

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
void ForceLimitsConstraint<TranslationalCoordinates_>::addLimit(const int limbId, const ForceLimit& normalGroundForceLimit) {
  limbNormalGroundForceLimits_[limbId] = normalGroundForceLimit;
}

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
ForceLimit ForceLimitsConstraint<TranslationalCoordinates_>::getLimit(const int limbId) const {
  if (limbNormalGroundForceLimits_.find(limbId) != limbNormalGroundForceLimits_.end()) {
    return limbNormalGroundForceLimits_.at(limbId);
  }
  return defaultNormalGroundForceLimits_;
}

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
bool ForceLimitsConstraint<TranslationalCoordinates_>::loadParameters(const TiXmlHandle& handle) {
  using namespace tinyxml_tools;
  TiXmlHandle forceDistributionHandle = getChildHandle(handle, "ContactForceDistribution");
  TiXmlHandle tempHandle = handle;

  // TODO change constraint xml structure
  if (getChildHandle(tempHandle, forceDistributionHandle, "Constraints")) {
    // Virtual force weights
    loadParameter(defaultNormalGroundForceLimits_.minForce_, tempHandle, "minimalNormalForce", 0.0);
    loadParameter(defaultNormalGroundForceLimits_.maxForce_, tempHandle, "maximalNormalForce", std::numeric_limits<double>::max());
  }

  if (getChildHandle(tempHandle, forceDistributionHandle, "LoadFactor", false)) {
    // Virtual torque weights
    loadParameter(defaultNormalGroundForceLimits_.maxForceLoadConstrained_, tempHandle, "maximalNormalForce",
                  std::numeric_limits<double>::max());
  }

  for (const auto& limb : wholeBody_.getLimbs()) {
    if (getChildHandle(tempHandle, forceDistributionHandle, limb->getName() + std::string("_Constraints"), false)) {
      // Virtual force weights
      loadParameter(limbNormalGroundForceLimits_[limb->getId()].minForce_, tempHandle, "minimalNormalForce",
                    defaultNormalGroundForceLimits_.minForce_);
      loadParameter(limbNormalGroundForceLimits_[limb->getId()].maxForce_, tempHandle, "maximalNormalForce",
                    defaultNormalGroundForceLimits_.maxForce_);
    }

    if (getChildHandle(tempHandle, forceDistributionHandle, limb->getName() + std::string("_LoadFactor"), false)) {
      // Virtual torque weights
      loadParameter(limbNormalGroundForceLimits_[limb->getId()].maxForceLoadConstrained_, tempHandle, "maximalNormalForce",
                    defaultNormalGroundForceLimits_.maxForceLoadConstrained_);
    }
  }

  return true;
}

template <cartesian::TranslationalCoordinates TranslationalCoordinates_>
void ForceLimitsConstraint<TranslationalCoordinates_>::print(std::ostream& out) const {
  out << "Default Force Limits: " << std::endl;
  out << defaultNormalGroundForceLimits_ << std::endl;
  for (const auto& gfl : limbNormalGroundForceLimits_) {
    out << "Limb (id:" << gfl.first << ") Force Limits: " << std::endl;
    out << gfl.second << std::endl;
  }
}

} /* namespace loco */
