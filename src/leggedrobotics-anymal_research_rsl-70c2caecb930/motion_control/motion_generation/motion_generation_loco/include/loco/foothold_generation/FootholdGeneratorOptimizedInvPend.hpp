/*
 * FootholdGeneratorOptimizedInvPend.hpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "FootholdGeneratorOptimizedConstraint.hpp"

class TiXmlHandle;

namespace loco {

class FootholdGeneratorOptimizedInvPend : public FootholdGeneratorOptimizedConstraint {
 public:
  using Base = FootholdGeneratorOptimizedConstraint;
  using Weight = Base::Weight;
  using Position2d = Base::Position2d;

  FootholdGeneratorOptimizedInvPend(WholeBody& wholeBody);

  ~FootholdGeneratorOptimizedInvPend() override = default;

 protected:

  //! Compute Hessian Q and linear term l of the cost function x'Qx + l'*x.
  bool setupCostFunction(unsigned int legId, const foothold_generator::FootholdPlan& plan) override;
};

} /* namespace loco */
