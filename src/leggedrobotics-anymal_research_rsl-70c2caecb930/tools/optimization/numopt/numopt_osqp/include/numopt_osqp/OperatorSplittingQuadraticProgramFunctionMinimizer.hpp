/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Yvain de Viragh, Marko Bjelonic, Dario Bellicoso
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich nor
 *     Carnegie Mellon University nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
* @file    OperatorSplittingQuadraticProgramFunctionMinimizer.hpp
* @author  Yvain de Viragh
* @date    May 18, 2018
* @brief
*/

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>

// numopt
#include "numopt_common/QuadraticProblemSolver.hpp"

// osqp
#include "osqp.h"

// stl
#include <iostream>


namespace numopt_osqp {

class OperatorSplittingQuadraticProgramFunctionMinimizer: public numopt_common::QuadraticProblemSolver
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OperatorSplittingQuadraticProgramFunctionMinimizer();
  virtual ~OperatorSplittingQuadraticProgramFunctionMinimizer() = default;
  virtual bool minimize(numopt_common::QuadraticProblem* problem,
                        numopt_common::Parameterization& params,
                        double& functionValue,
                        unsigned int nonlinIter = 0) override;

  //! Set to true to allow for external warm starting.
  virtual void setExternalWarmStart(bool externalWarmStart);

  //! Set to true if consistency of problem dimensions should always be checked.
  virtual void setCheckProblemValidity(bool checkProblemValidity);

 protected:

  //! Dimension checks.
  virtual bool isProblemValid() const;

  //! True, if external warm start is allowed. Default is false.
  bool externalWarmStart_;

  //! True, if problem validity should always be checked. Default is false.
  bool checkProblemValidity_;

  //! The cost function Hessian.
  numopt_common::SparseMatrix Q_;

  //! The cost function linear term,
  numopt_common::Vector c_;

  //! The equality constraints Jacobian.
  numopt_common::SparseMatrix A_;

  //! The equality constraint target values.
  numopt_common::Vector b_;

  //! The inequality constraints Jacobian.
  numopt_common::SparseMatrix D_;

  //! The inequality constraint minimum values.
  numopt_common::Vector d_;

  //! The inequality constraint maximum values.
  numopt_common::Vector f_;

  //! The lower bound values.
  numopt_common::Vector l_;

  //! The upper bound values.
  numopt_common::Vector u_;

  //! The optimization parameters.
  numopt_common::Params p_;

  //! Problem dimensions.
  c_int solutionDimension_;
  c_int numEq_;
  c_int numIneq_;
  c_int numBounds_;
  c_int numConstr_;

};

} /* namespace numopt_osqp */
