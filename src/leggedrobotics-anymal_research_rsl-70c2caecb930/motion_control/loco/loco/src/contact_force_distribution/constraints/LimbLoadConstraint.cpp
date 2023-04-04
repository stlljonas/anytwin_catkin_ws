/*
 * LimbLoadConstraint.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#include "loco/contact_force_distribution/constraints/LimbLoadConstraint.hpp"

namespace loco {

LimbLoadConstraint::LimbLoadConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain)
    : ConstraintInterface("LimbLoadConstraint", wholeBody, terrain) {}

bool LimbLoadConstraint::updateConstraint(const LimbInfos& limbInfos) {
  /*
   * The contact force distribution is calculated
   * twice, once without the load factor equality constraint and then
   * including the equality constraint.
   * For each leg with user defined load constraints, we add equality constraints of the form
   * f_i = loadFactor * f_i_previous, with f_i the contact force of leg i and f_i_previous
   * the contact force of leg i at the optimization without user defined leg load constraints.
   */
  constexpr unsigned int m = cartesian::forceVectorSize;
  const unsigned int nConstraints = m * limbInfos.getNumLimbsWithActiveLegLoadConstraint();
  unsigned int rowOffset = 0;
  globalEqConJacobian_.conservativeResize(nConstraints, limbInfos.getNumLimbsInForceDistribution() * m);
  eqConTargetValues_.conservativeResize(nConstraints);
  Eigen::MatrixXd limbLoadConstraint(m, limbInfos.getNumLimbsInForceDistribution() * m);

  for (const auto& limb : wholeBody_.getLimbs()) {
    if (limbInfos[limb].isLoadConstraintActive_) {
      limbLoadConstraint.block<m, m>(0, limbInfos[limb].limbStartIndexInSolutionVector_) = Eigen::MatrixXd::Identity(m, m);
      globalEqConJacobian_.middleRows(rowOffset, m) = limbLoadConstraint.sparseView();
      eqConTargetValues_.segment(rowOffset, m) =
          limb->getLoadFactor() * solutionWithoutActiveLoadConstraint_.segment(limbInfos[limb].limbStartIndexInSolutionVector_, m);
      rowOffset += m;
    }
  }
  return true;
}

} /* namespace loco */
