/*
 * LimbLoadConstraint.hpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/contact_force_distribution/constraints/ConstraintInterface.hpp"

// Eigen
#include <Eigen/Core>

namespace loco {

class LimbLoadConstraint : public ConstraintInterface {
 public:
  // Constructor
  LimbLoadConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain);

  //! Destructor
  ~LimbLoadConstraint() override = default;

  /** Update the jacobian and target values of the constraint
   * @param limbInfos  Distribution specific limb data
   */
  bool updateConstraint(const LimbInfos& limbInfos) override;

  /** Sets the current solution without applying the load constraint
   * @param solutionWithoutActiveLoadConstraint solution without active load constraint
   */
  void setSolutionWithoutActiveLoadConstraint(const Eigen::VectorXd& solutionWithoutActiveLoadConstraint) {
    solutionWithoutActiveLoadConstraint_ = solutionWithoutActiveLoadConstraint;
  }

 protected:
  Eigen::VectorXd solutionWithoutActiveLoadConstraint_;
};

} /* namespace loco */
