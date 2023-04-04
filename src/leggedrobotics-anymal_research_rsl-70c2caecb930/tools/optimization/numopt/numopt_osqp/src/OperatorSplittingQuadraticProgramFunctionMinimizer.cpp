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
* @file    OperatorSplittingQuadraticProgramFunctionMinimizer.cpp
* @author  Yvain de Viragh
* @date    May 18, 2018
* @brief
*/

#include "numopt_osqp/OperatorSplittingQuadraticProgramFunctionMinimizer.hpp"

namespace numopt_osqp {

OperatorSplittingQuadraticProgramFunctionMinimizer::OperatorSplittingQuadraticProgramFunctionMinimizer()
: externalWarmStart_(false),
  checkProblemValidity_(false),
  solutionDimension_(0),
  numEq_(0),
  numIneq_(0),
  numBounds_(0),
  numConstr_(0)
{

}

bool OperatorSplittingQuadraticProgramFunctionMinimizer::minimize(numopt_common::QuadraticProblem* problem,
                                                                  numopt_common::Parameterization& params,
                                                                  double& functionValue,
                                                                  unsigned int nonlinIter)
{

  /*
   * Problem form given by numopt_common:
   *
   * x_opt = argmin 1/2*x'*Q*x + c'*x
   *         subj. to  A*x == b,  d <= D*x <= f,  l <= x <= u
   *
   * Problem form expected by osqp:
   *
   * x_opt = argmin 1/2*x'*Qhat*x + chat'*x
   *         subj. to lhat <= Ahat*x <= uhat
   *
   * where x is of size nx1 and Ahat is of size mxn.
   *
   */

  bool success = true;

  if (!problem->getQuadraticObjectiveFunctionPtr()->getGlobalHessian(Q_, params)) {
    return false;
  }

  if (!problem->getQuadraticObjectiveFunctionPtr()->getLinearTerm(c_)) {
    return false;
  }

  if (!problem->getFunctionConstraintsPtr()->getGlobalEqualityConstraintJacobian(A_, params)) {
    return false;
  }

  if (!problem->getFunctionConstraintsPtr()->getEqualityConstraintTargetValues(b_)){
    return false;
  }

  if (!problem->getFunctionConstraintsPtr()->getGlobalInequalityConstraintJacobian(D_, params)) {
    return false;
  }

  if (!problem->getFunctionConstraintsPtr()->getInequalityConstraintMinValues(d_)){
    return false;
  }

  if (!problem->getFunctionConstraintsPtr()->getInequalityConstraintMaxValues(f_)){
    return false;
  }

  if (!problem->getFunctionConstraintsPtr()->getGlobalBoundConstraintMinValues(l_)){
    return false;
  }

  if (!problem->getFunctionConstraintsPtr()->getGlobalBoundConstraintMaxValues(u_)){
    return false;
  }

  p_ = params.getParams();
  params.setIdentity(p_); // TODO: Necessary?

  // Compute number of states and constraints.
  solutionDimension_     = Q_.rows();
  numEq_                 = A_.size() == 0 ? 0 : A_.rows();
  numIneq_               = D_.size() == 0 ? 0 : D_.rows();
  numBounds_             = (l_.size() == 0 && u_.size() == 0) ? 0u : solutionDimension_;
  numConstr_             = numEq_ + numIneq_ + numBounds_;
  const c_int nnzConstr  = A_.nonZeros() + D_.nonZeros() + numBounds_;
  const c_int nnzHessian = Q_.nonZeros();

  if (solutionDimension_ == 0) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] Number of states is zero! Nothing to do.\n";
    return true;
  }

  // Check problem validity.
  if(checkProblemValidity_) {
    if(!isProblemValid()) {
      throw std::runtime_error("[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] Invalid problem.\n");
    }
  } else {
    assert(isProblemValid());
  }

  // Hessian.
  Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int> Qhat(Q_);
  Qhat.makeCompressed();

  // Linear objective term.
  Eigen::Matrix<c_float, Eigen::Dynamic, 1u> chat(solutionDimension_);
  if(c_.size() == 0) {
    chat.setZero();
  } else {
    chat = c_;
  }

  // Helper matrix for constructing Ahat.
  Eigen::SparseMatrix<c_float, Eigen::RowMajor> auxAhat(numConstr_, solutionDimension_);
  auxAhat.reserve(nnzConstr);

  // Fill auxA.
  if(numEq_ != 0) {
    auxAhat.topRows(numEq_) = A_;
  }
  if(numIneq_ != 0) {
    auxAhat.middleRows(numEq_,numIneq_) = D_;
  }
  if(numBounds_ != 0) {
    Eigen::SparseMatrix<c_float, Eigen::RowMajor> identityMatrix(numBounds_,numBounds_);
    identityMatrix.setIdentity();
    auxAhat.bottomRows(numBounds_) = identityMatrix;
  }

  // Construct Ahat form auxAhat.
  Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int> Ahat(auxAhat);
  Ahat.makeCompressed();

  // Constraint vectors.
  Eigen::Matrix<c_float, Eigen::Dynamic, 1u> lhat(numConstr_, 1u);
  Eigen::Matrix<c_float, Eigen::Dynamic, 1u> uhat(numConstr_, 1u);

  // Equality constraint vector.
  lhat.topRows(numEq_) = b_;
  uhat.topRows(numEq_) = b_;

  // Inequality constraint vectors.
  if((d_.rows() == 0) && (numIneq_ != 0)) {
    lhat.middleRows(numEq_,numIneq_).setConstant(-OSQP_INFTY);
  } else {
    lhat.middleRows(numEq_,numIneq_) = d_;
  }

  if((f_.rows() == 0) && (numIneq_ != 0)) {
    uhat.middleRows(numEq_,numIneq_).setConstant(OSQP_INFTY);
  } else {
    uhat.middleRows(numEq_,numIneq_) = f_;
  }

  // Bound vectors.
  if((l_.rows() == 0) && (numBounds_ != 0)) {
    lhat.bottomRows(numBounds_).setConstant(-OSQP_INFTY);
  } else {
    lhat.bottomRows(numBounds_) = l_;
  }

  if((u_.rows() == 0) && (numBounds_ != 0)) {
    uhat.bottomRows(numBounds_).setConstant(OSQP_INFTY);
  } else {
    uhat.bottomRows(numBounds_) = u_;
  }

  // Problem settings and structs.
  OSQPSettings* settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  OSQPWorkspace* workspace;
  OSQPData* data;

  // Define Solver settings as default.
  osqp_set_default_settings(settings);
  settings->verbose = 0; // 1 means verbose output is printed, 0 not.

  // Set problem data.
  data = (OSQPData *)c_malloc(sizeof(OSQPData));
  data->n = solutionDimension_;
  data->m = numConstr_;
  data->P = csc_matrix(data->n, data->n, Qhat.nonZeros(), Qhat.valuePtr(), Qhat.innerIndexPtr(), Qhat.outerIndexPtr());
  data->q = chat.data();
  data->A = csc_matrix(data->m, data->n, Ahat.nonZeros(), Ahat.valuePtr(), Ahat.innerIndexPtr(), Ahat.outerIndexPtr());
  data->l = lhat.data();
  data->u = uhat.data();

  // Setup workspace.
  workspace = osqp_setup(data, settings);

  // Set initial guess, if external warm start is turned on.
  if(externalWarmStart_) {
    Eigen::Matrix<c_float, Eigen::Dynamic, 1u> xInit = params.getParams();

    // Make sure dimensions match.
    if(xInit.size() == solutionDimension_) {
      osqp_warm_start_x(workspace, xInit.data());
    }
  }

  // Solve Problem.
  if(osqp_solve(workspace) != 0) {
    std::cerr << "\n[OperatorSplittingQuadraticProgramFunctionMinimizer.minimize] osqp_solve returned an error. Status: " << workspace->info->status << ".";
    success = false;
  } else if(workspace->info->status_val != OSQP_SOLVED) {
    std::cout << "\n[OperatorSplittingQuadraticProgramFunctionMinimizer.minimize] Optimization failed. Status: " << workspace->info->status << "." << std::endl;
    success = false;
  }

  params.getParams() = Eigen::Map<Eigen::VectorXd>(workspace->solution->x, solutionDimension_, 1);
  problem->setCurrentBestSolution(params);

  functionValue = (0.5 * params.getParams().transpose() * Q_ * params.getParams() + chat.transpose() * params.getParams())(0); // Workaround to convert eigen scalar to double.

  // Cleanup.
  if(osqp_cleanup(workspace) != 0) {
    std::cerr << "[OperatorSplittingQuadraticProgramFunctionMinimizer.minimize] Something in osqp_cleanup went wrong."
        " This should be investigated, as it might be indicating a memory leak or similar!";
    success = false;
  }
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return success;
}


void OperatorSplittingQuadraticProgramFunctionMinimizer::setExternalWarmStart(bool externalWarmStart) {
  externalWarmStart_ = externalWarmStart;
}


void OperatorSplittingQuadraticProgramFunctionMinimizer::setCheckProblemValidity(bool checkProblemValidity) {
  checkProblemValidity_ = checkProblemValidity;
}


bool OperatorSplittingQuadraticProgramFunctionMinimizer::isProblemValid() const {
  bool isValid = true;

  // Dimension consistency checks.
  if(!(Q_.cols() == solutionDimension_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Hessian is not square!\n";
    isValid = false;
  }
  if(!(c_.size() == 0 || c_.size() == solutionDimension_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Linear objective term has inconsistent size!\n";
    isValid = false;
  }
  if(!(A_.size() == numEq_ * solutionDimension_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Linear equality constraint matrix has inconsistent size!\n";
    isValid = false;
  }
  if(!(b_.size() == numEq_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Linear equality constraint vector has inconsistent size!\n";
    isValid = false;
  }
  if(!(D_.size() == numIneq_ * solutionDimension_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Linear inequality constraint matrix has inconsistent size!\n";
    isValid = false;
  }
  if(!(d_.size() == 0 || d_.size() == numIneq_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Linear inequality constraint minimum vector has inconsistent size!\n";
    isValid = false;
  }
  if(!(f_.size() == 0 || f_.size() == numIneq_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Linear inequality constraint maximum vector has inconsistent size!\n";
    isValid = false;
  }
  if(!(l_.size() == 0 || l_.size() == solutionDimension_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Lower bounds have inconsistent size!\n";
    isValid = false;
  }
  if(!(u_.size() == 0 || u_.size() == solutionDimension_)) {
    std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
        "Upper bounds have inconsistent size!\n";
    isValid = false;
  }

  if(numIneq_ != 0) {
    if((d_.size() == 0 && f_.size() == 0)) {
      std::cout << "[OperatorSplittingQuadraticProgramFunctionMinimizer::minimize] "
          "Linear inequality constraint minimum and maximum vectors can not both be empty!\n";
      isValid = false;
    }
  }

  return isValid;
}


} /* namespace numopt_osqp */
