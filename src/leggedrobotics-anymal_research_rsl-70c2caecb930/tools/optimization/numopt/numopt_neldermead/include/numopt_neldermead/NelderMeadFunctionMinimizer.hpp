/*
 * NelderMeadFunctionMinimizer.hpp
 *
 *  Created on: Aug 8, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

#include "numopt_common/QuadraticProblemSolver.hpp"

namespace numopt_neldermead {

class NelderMeadFunctionMinimizer: public numopt_common::QuadraticProblemSolver {
 public:
  NelderMeadFunctionMinimizer();
  virtual ~NelderMeadFunctionMinimizer();

  virtual bool minimize(numopt_common::QuadraticProblem* problem,
                        numopt_common::Parameterization& p,
                        double& functionValue);

};

} /* namespace numopt_neldermead */
