/*
 * ActuatorEffortConstraint.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#include "loco/contact_force_distribution/constraints/ActuatorEffortConstraint.hpp"

#include <tinyxml_tools/tinyxml_tools.hpp>

#include <limits>

namespace loco {

ActuatorEffortConstraint::ActuatorEffortConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain)
    : ConstraintInterface("ActuatorEffortConstraint", wholeBody, terrain),
      actuatorEffortLimit_{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()} {}

bool ActuatorEffortConstraint::updateConstraint(const LimbInfos& limbInfos) {
  auto numActuatorsInDistribution = 0;
  for (const auto& limb : wholeBody_.getLimbs()) {
    if (limbInfos[limb].isPartOfForceDistribution_) {
      // Joint effort limits.
      numActuatorsInDistribution += limb->getNumDofLimb();
    }
  }

  globalIneqConJacobian_.resize(numActuatorsInDistribution, cartesian::forceVectorSize * limbInfos.getNumLimbsInForceDistribution());
  ineqConMaxValues_.resize(numActuatorsInDistribution);
  ineqConMaxValues_.setConstant(std::numeric_limits<double>::max());

  ineqConMinValues_.resize(numActuatorsInDistribution);
  ineqConMinValues_.setConstant(std::numeric_limits<double>::lowest());

  unsigned int rowOffset = 0;
  // Joint effort limits.
  for (const auto& limb : wholeBody_.getLimbs()) {
    if (limbInfos[limb].isPartOfForceDistribution_) {
      const auto numDofLimb = limb->getNumDofLimb();
      const TranslationJacobianLimb& limbJacobianInBaseFrame =
          limb->getEndEffector().getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame();

      /* g = tau + J_t * F
       * tau_max >= g - J_t * F
       * tau_min <= g - J_t * F
       * g - tau_max <= J_t * F <= g - tau_min
       */
      Eigen::MatrixXd middleJac;
      middleJac.setZero(numDofLimb, cartesian::forceVectorSize * limbInfos.getNumLimbsInForceDistribution());
      middleJac.middleCols(limbInfos[limb].limbStartIndexInSolutionVector_, cartesian::forceVectorSize) =
          limbJacobianInBaseFrame.transpose();
      globalIneqConJacobian_.middleRows(rowOffset, numDofLimb) = middleJac.sparseView();

      ineqConMinValues_.segment(rowOffset, numDofLimb) =
          limb->getLimbStateMeasured().getGravityJointTorques() - Eigen::VectorXd::Constant(numDofLimb, actuatorEffortLimit_.maxEffort_);
      ineqConMaxValues_.segment(rowOffset, numDofLimb) =
          limb->getLimbStateMeasured().getGravityJointTorques() - Eigen::VectorXd::Constant(numDofLimb, actuatorEffortLimit_.minEffort_);
      rowOffset += numDofLimb;
    }
  }

  return true;
}

bool ActuatorEffortConstraint::loadParameters(const TiXmlHandle& handle) {
  const TiXmlHandle forceDistributionHandle = tinyxml_tools::getChildHandle(handle, "ContactForceDistribution");

  TiXmlHandle actuatorLimitsHandle = handle;
  if (!tinyxml_tools::getChildHandle(actuatorLimitsHandle, forceDistributionHandle, "ActuatorLimits")) {
    return false;
  }

  // Virtual force weights
  tinyxml_tools::loadParameter(actuatorEffortLimit_.minEffort_, actuatorLimitsHandle, "min", std::numeric_limits<double>::lowest());
  tinyxml_tools::loadParameter(actuatorEffortLimit_.maxEffort_, actuatorLimitsHandle, "max", std::numeric_limits<double>::max());

  return true;
}

} /* namespace loco */
