/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Christian Gehring, Stelian Coros
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
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich nor
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
* @file    SQPFunctionMinimizer.cpp
* @author  Stelian Coros, Christian Gehring, Peter Fankhauser
* @date    Aug 16, 2015
*/

// numopt sqp
#include <numopt_sqp/SQPFunctionMinimizer.hpp>

// numopt common
#include <numopt_common/NonlinearObjectiveFunction.hpp>
#include <numopt_common/numopt_assert_macros.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

// stl
#include <fstream>
#include <iostream>

namespace numopt_sqp {

using namespace numopt_common;

SQPFunctionMinimizer::SQPFunctionMinimizer(
    std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver,
    int maxIterations,
    double solveResidual,
    unsigned int maxLineSearchIterations,
    double solveFunctionValue,
    bool printOutput,
    bool checkConstraints)
  :  qpSolver_(qpSolver),
     qpProblem_(std::shared_ptr<QuadraticObjectiveFunction>(new QuadraticObjectiveFunction),
                std::shared_ptr<LinearFunctionConstraints>(new LinearFunctionConstraints)),
     maxIterations_(maxIterations),
     solveResidual_(solveResidual),
     maxLineSearchIterations_(maxLineSearchIterations),
     solveFunctionValue_(solveFunctionValue),
     printOutput_(printOutput),
     checkConstraints_(checkConstraints),
     interrupt_(false)
{
}

void SQPFunctionMinimizer::registerOptimizationStepCallback(std::function<void(const size_t,
                                        const numopt_common::Parameterization&, const double, const bool)> callback) {
  optimizationStepCallback_ = callback;
}

void SQPFunctionMinimizer::registerOptimizationStepInitCallback(std::function<void(const numopt_common::Parameterization&)> callback) {
  optimizationStepInitCallback_ = callback;
}

void SQPFunctionMinimizer::setSolveResidual(double solveResidual) {
  solveResidual_ = solveResidual;
}

void SQPFunctionMinimizer::setPrintOutput(bool isPrinting) {
  printOutput_ = isPrinting;
}

void SQPFunctionMinimizer::setCheckConstraints(bool checkConstraints) {
  checkConstraints_ = checkConstraints;
}

void SQPFunctionMinimizer::setMaxIterations(int maxIterations) {
  maxIterations_ = maxIterations;
}

/**
  min f(p) subject to the constraints...
*/
bool SQPFunctionMinimizer::minimize(ConstrainedNonlinearProblem* problem,
                                    Parameterization& params,
                                    double &functionValue,
                                    const std::string& fileName,
                                    bool printProblemToFile){
  if (interrupt_) { return false; }

  if (printOutput_) {
    printf("Starting SQP...\n");
  }

  const int nLocalParameters = params.getLocalSize();
  Delta dp = Delta::Zero(nLocalParameters);
  Vector objectiveGradient;
  SparseMatrix objectiveHessian;
  SparseMatrix equalityConstraintsJacobian;
  Vector equalityConstraintsTargetValues;
  Vector inequalityConstraintsMinValues;
  SparseMatrix inequalityConstraintsJacobian;
  Vector inequalityConstraintsMaxValues;
  Vector lowerBounds;
  Vector upperBounds;

  problem->setCurrentBestSolution(params);

  std::ofstream file;
  if (printProblemToFile) {
    file.open(fileName);
    if (!file) {
      std::cout << "[SQPFunctionMinimizer::minimize] Failed to open file.";
      printProblemToFile = false;
    }
  }

  // Iterate - like Newton
  bool optimizationConverged = false;
  double dp_norm = 0.0;
  unsigned int i;
  for (i=0; i<maxIterations_; ++i) {
    // Update gradient and hessian of objective function.
    if (printOutput_) { printf("Updating gradient.\n"); }

    if (interrupt_) { return false; }
    if (optimizationStepInitCallback_) optimizationStepInitCallback_(params);

    if (interrupt_) { return false; }
    if (!problem->getObjectiveFunctionPtr()->getLocalGradient(objectiveGradient, params, true)) {
      return false;
    }

    if (printOutput_) { printf("Updating hessian.\n"); }
    if (interrupt_) { return false; }
    if (!problem->getObjectiveFunctionPtr()->getLocalHessian(objectiveHessian, params, false)) {
      return false;
    }

    if (printOutput_) { printf("Updating eq. Jacobian.\n"); }
    if (interrupt_) { return false; }
    if (!problem->getFunctionConstraintsPtr()->getLocalEqualityConstraintJacobian(equalityConstraintsJacobian, params, true)) {
      return false;
    }

    if (printOutput_) { printf("Updating eq. target values.\n"); }
    if (interrupt_) { return false; }
    if (!problem->getFunctionConstraintsPtr()->getEqualityConstraintTargetValues(equalityConstraintsTargetValues)) {
      return false;
    }

    if (printOutput_) { printf("Updating in. eq. min values\n"); }
    if (interrupt_) { return false; }
    if (!problem->getFunctionConstraintsPtr()->getInequalityConstraintMinValues(inequalityConstraintsMinValues)) {
      return false;
    }

    if (printOutput_) { printf("Updating in. eq. jacobian\n"); }
    if (interrupt_) { return false; }
    if (!problem->getFunctionConstraintsPtr()->getLocalInequalityConstraintJacobian(inequalityConstraintsJacobian, params, false)) {
      return false;
    }

    if (printOutput_) { printf("Updating in. eq. max values\n"); }
    if (interrupt_) { return false; }
    if (!problem->getFunctionConstraintsPtr()->getInequalityConstraintMaxValues(inequalityConstraintsMaxValues)) {
      return false;
    }

    if (interrupt_) { return false; }
    if (!problem->getFunctionConstraintsPtr()->getGlobalBoundConstraintMinValues(lowerBounds)) {
      return false;
    }

    if (interrupt_) { return false; }
    if (!problem->getFunctionConstraintsPtr()->getGlobalBoundConstraintMaxValues(upperBounds)) {
      return false;
    }

    if (interrupt_) { return false; }
    if (printOutput_) { printf("Computing search direction ..\n"); }

    // Find the direction to step at
    if (!computeSearchDirection(problem,
                           objectiveHessian,
                           objectiveGradient,
                           params,
                           dp,
                           equalityConstraintsJacobian,
                           equalityConstraintsTargetValues,
                           inequalityConstraintsMinValues,
                           inequalityConstraintsJacobian,
                           inequalityConstraintsMaxValues,
                           lowerBounds,
                           upperBounds,
                           params.getLocalSize() == params.getGlobalSize(),
                           i)) {
      if (printOutput_) {
        printf("Could not compute search direction!\n");
      }

      if (printProblemToFile && !printToFile(file, i)) {
        std::cout << "[SQPFunctionMinimizer::minimize] Failed to print to file!";
      }

      if (printProblemToFile) { file.close(); }
      return false;
    }

    if (printProblemToFile && !printToFile(file, i)) {
      std::cout << "[SQPFunctionMinimizer::minimize] Failed to print to file!";
    }

    if (interrupt_) { return false; }
    if (printOutput_) {
      printf("Computing function value ..\n");
    }
    double currentFunctionValue = 1.0e20;
    problem->getObjectiveFunctionPtr()->computeValue(currentFunctionValue, params, true);

    if (printOutput_) {
      printf("Starting iteration %d. Initial function value: %10.10lf. Search direction norm: %lf. Gradient norm: %lf\n", i,
             currentFunctionValue,
             dp.norm(),
             objectiveGradient.norm());
    }

    // Check if full step is within tolerance.
    dp_norm = dp.norm();
    if (dp_norm < solveResidual_ || currentFunctionValue < solveFunctionValue_)  {
      optimizationConverged = true;
    }

    if (interrupt_) { return false; }

    // Do a line search to appropriately scale the step length.
    double stepSize;
    if (!doLineSearch(stepSize, problem, params, dp, i)) {
      if (printOutput_) {
        printf("\tCancelling iteration: No better function value found\n");
      }
      optimizationConverged = true;
    }

    // Check if scaled step is within tolerance.
    if (dp_norm*stepSize < solveResidual_)  {
      optimizationConverged = true;
    }

    // Update solution: p(i+1) = p(i) + alpha*dp
    params.plus(params.getParams(), params.getParams(), stepSize*dp);
    problem->setCurrentBestSolution(params);

    if (interrupt_) { return false; }
    if (optimizationStepCallback_) optimizationStepCallback_(i, params, currentFunctionValue, false);

    if (optimizationConverged) {
      break;
    }
  }

  // output function value
  if(!problem->getObjectiveFunctionPtr()->computeValue(functionValue, params, true)) {
    return false;
  }

  // output solution
  if (printOutput_) {
    printf("===== Done SQP optimization =====\n");
    if (optimizationConverged) {
      printf("   Converged in %d iterations!\n", i);
    }
    else {
      printf("   Did NOT converge!\n");
    }
    printf("   Final function value: %10.10lf\n", functionValue);
    printf("   Final Gradient norm: %10.10lf\n", objectiveGradient.norm());
    printf("   Final search direction norm: %10.10lf\n", dp_norm);
    problem->getFunctionConstraintsPtr()->printConstraintErrors(params);
  }

  if (optimizationStepCallback_) { optimizationStepCallback_(i, params, functionValue, true); }

  if (printProblemToFile) { file.close(); }

  return optimizationConverged;
}

bool SQPFunctionMinimizer::computeSearchDirection(numopt_common::ConstrainedNonlinearProblem* problem,
                                                  const SparseMatrix& hessian,
                                                  const Vector& gradient,
                                                  const Parameterization &p,
                                                  Vector &dp,
                                                  const SparseMatrix& A,
                                                  const Vector& b,
                                                  const Vector& d,
                                                  const SparseMatrix& C,
                                                  const Vector& f,
                                                  const Vector& l,
                                                  const Vector& u,
                                                  bool useParameterBounds,
                                                  unsigned int iter) {


  /**
    * We solve the system of equations
    *   1/2 dp'*H*dp + grad*dp == 0
    * in a least squares sense while maintaining the constraints A*p = b and d <= C*p <= f.
    * This means we want to solve
    *  min 1/2 dp'*H*dp + grad*dp,
    *   s.t. A*(p + dp) = b,         ->   A*dp = b - A*p
    *        d <= C*(p + dp) <= f    ->   d-C*p <= C*dp <= f-C*p
    *        l <= p + dp <=  u       ->   l-p <= dp <= u-p
    * For the QP solver we need the following canonical form:
    *    min 1/2 x' Q x + c' x s. t. A x = b, d <= Cx <= f, and l <= x <= u
    */

  Vector fMinusCp;
  Vector dMinusCp;
  if (interrupt_) { return false; }
  if (C.size() > 0) {
    Vector Cp;
    if(!problem->getFunctionConstraintsPtr()->getInequalityConstraintValues(Cp, p, false)) {
      return false;
    }
    fMinusCp = f - Cp;
    dMinusCp = d - Cp;
  }

  if (interrupt_) { return false; }
  Vector bMinusAp;
  if (A.size() > 0) {
    Vector Ap;
    if(!problem->getFunctionConstraintsPtr()->getEqualityConstraintValues(Ap, p, false)) {
      return false;
    }

//    std::cout << "Aglobal rows: " << Aglobal.rows() << " cols:" << Aglobal.cols() << std::endl;
//    std::cout << "Params:\n" << p.getParams().size() << std::endl;
//    std::cout << "b:\n" << b.size() << std::endl;
    bMinusAp = b - Ap;
//    std::cout << "bMinusAp\n" << bMinusAp << std::endl;
  }

  Vector lMinusp;
  Vector uMinusp;

  if (useParameterBounds) {
    // this is only possible if local size == global size
    lMinusp = l-p.getParams();
    uMinusp = u-p.getParams();

    /*  const Eigen::IOFormat eigenFormat(2, 0, ",","\n", "[", "]");
        std::cout << "useParameterBounds" << std::endl;
        std::cout << "lowerBounds:" << std::endl;
        std::cout << l.format(eigenFormat) << std::endl;
        std::cout << "upperBounds:" << std::endl;
        std::cout << u.format(eigenFormat) << std::endl;
    */
  }
  else {
    lMinusp = Vector::Ones(p.getLocalSize())*-std::numeric_limits<double>::max();
    uMinusp = Vector::Ones(p.getLocalSize())*std::numeric_limits<double>::max();
  }

  /**
   * We solve the system of equations
   *   1/2 dp'*H*dp + grad*dp == 0
   * in a least squares sense while maintaining the constraints A*p = b and d <= C*p <= f.
   * This means we want to solve
   *  min 1/2 dp'*H*dp + grad*dp,
   *   s.t. A*(p + dp) = b,         ->   A*dp = b - A*p
   *        d <= C*(p + dp) <= f    ->   d-C*p <= C*dp <= f-C*p
   *        l <= p + dp <=  u       ->   l-p <= dp <= u-p
   * For the QP solver we need the following canonical form:
   *    min 1/2 x' Q x + c' x s. t. A x = b, d <= Cx <= f, and l <= x <= u
   *
   * x = dp
   * H = hessian
   * c = gradient
   * A = A
   * b = bMinusAp
   * C = C
   * d = dMinusCp
   * f = fMinusCp
   * l = minMinusdp
   * u = uMinusdp
  */

//   std::cout << "SQPFunctionMinimizer::computeSearchDirection C.rows: " << C.rows() << " C.cols: "<< C.cols() << std::endl;

   double dpFunctionValue = 0.0;
   if (interrupt_) { return false; }
   qpProblem_.getQuadraticObjectiveFunctionPtr()->setGlobalHessian(hessian);
   qpProblem_.getQuadraticObjectiveFunctionPtr()->setLinearTerm(gradient);
   qpProblem_.getLinearFunctionConstraintsPtr()->setGlobalEqualityConstraintJacobian(A);
   qpProblem_.getLinearFunctionConstraintsPtr()->setEqualityConstraintTargetValues(bMinusAp);
   qpProblem_.getLinearFunctionConstraintsPtr()->setGlobalInequalityConstraintJacobian(C);
   qpProblem_.getLinearFunctionConstraintsPtr()->setInequalityConstraintMinValues(dMinusCp);
   qpProblem_.getLinearFunctionConstraintsPtr()->setInequalityConstraintMaxValues(fMinusCp);
   qpProblem_.getLinearFunctionConstraintsPtr()->setGlobalBoundConstraintMinValues(lMinusp);
   qpProblem_.getLinearFunctionConstraintsPtr()->setGlobalBoundConstraintMaxValues(uMinusp);
   ParameterizationIdentity qpParams(dp);
   bool success = qpSolver_->minimize(&qpProblem_, qpParams, dpFunctionValue, iter);
   dp = qpParams.getParams();

  if (success && printOutput_) {
    printf("  ==> Search direction: dot product between gradient and search direction: %lf\n", -(gradient.dot(dp)));
  }
  if (!success){
    printf("Failed QP solve in SQP - printing system and returning zero stepping direction\n");
    dp.setZero();
  }

  // Make sure the solution always stays within the constraints.
  if (success && checkConstraints_) {
    Parameterization* pnext = p.clone();
    pnext->plus(pnext->getParams(), p.getParams(), dp);

    // p must satisfy A*p = b after the solve...
    if (A.cols() == p.getLocalSize()){
      SparseMatrix Aglobal;
//      p.transformLocalToGlobal(Aglobal, p.getParams(), A);
      Aglobal = A;
      Vector ap = Aglobal*p.getParams();
      Vector apNext = Aglobal*pnext->getParams();
      for (unsigned int i=0; i<ap.size(); i++){
        if (std::abs(ap(i) - b(i)) >= 0.0000001 || std::abs(apNext(i) - b(i)) >= 0.0000001)
          printf("++++++++ SQP: equality constraint %d: p: %.10lf, p+dp: %.10lf, constraint value: %lf\n", i, ap(i), apNext(i), b(i));
      }
    }

    // p must satisfy d <= C*p <= f
    if (C.cols() == p.getLocalSize()){
      SparseMatrix Cglobal;
//      p.transformLocalToGlobal(Cglobal, p.getParams(), C);
      Cglobal = C;
      Vector cp = Cglobal*p.getParams();
      Vector cpNext = Cglobal*pnext->getParams();

      for (uint i=0; i<cp.size(); i++){
//        assert("p(t) within inequality constraints" && cp[i] <= f->at(i) + 0.0000001);
        if (cp(i) > f(i) + 0.0000001 || cpNext[i] > f(i) + 0.0000001)
          printf("++++++++ SQP: inequality constraint %d: p: %.10lf, p+dp: %.10lf, min val: %.10lf\n", i, cp(i), cpNext(i), f(i));

        if (cp(i) < d(i) - 0.0000001 || cpNext(i) < d(i) - 0.0000001)
          printf("++++++++ SQP: inequality constraint %d: p: %.10lf, p+dp: %.10lf, min val: %.10lf\n", i, cp(i), cpNext(i), d(i));
      }
    }

    // p must satisfy the bound constraints lo <= p <= hi
    if (l.size() == p.getGlobalSize()){
      for (uint i=0; i<p.getGlobalSize(); i++){
        if (l(i) == u(i))
          continue;
//        assert("p(t) within inequality constraints" && cp[i] <= f->at(i) + 0.0000001);
        if (p.getParams()(i) < l(i) - 0.0000001 || pnext->getParams()(i) < l(i) - 0.0000001)
          printf("++++++++ SQP: min bound constraint %d: p: %.10lf, p+dp: %.10lf, lo: %.10lf\n", i, p.getParams()(i), pnext->getParams()(i), l(i));

        if (p.getParams()(i) > u(i) + 0.0000001 || pnext->getParams()(i) > u(i) + 0.0000001)
          printf("++++++++ SQP: max bound constraint %d: p: %.10lf, p+dp: %.10lf, hi: %.10lf\n", i, p.getParams()(i), pnext->getParams()(i), u(i));
      }
    }
  }



  return success;
}

bool SQPFunctionMinimizer::doLineSearch(double& stepSize,
                                        ConstrainedNonlinearProblem* const problem,
                                        Parameterization& p,
                                        const Vector& dp,
                                        unsigned int)
{
  stepSize = 1.0;

  if (maxLineSearchIterations_ <= 1u) {
    return true;
  }

  double initialValue = 1.0e20;
  if (!problem->getObjectiveFunctionPtr()->computeValue(initialValue, p)) {
    return false;
  }

  Parameterization* pc = p.clone();

  for (int j=0; j<maxLineSearchIterations_; ++j)
  {
    //pc = p + stepSize*dp;
    Vector delta = stepSize*dp;
    pc->plus(pc->getParams(), p.getParams(), delta);

    double newLineSearchValue = 1.0e20;
    problem->getObjectiveFunctionPtr()->computeValue(newLineSearchValue, *pc);

    if (printOutput_) {
      printf("\t--> LINE SEARCH iteration %d: stepSize is %10.10lf, function value is: %10.10lf\n", j, stepSize, newLineSearchValue);
    }

    if (newLineSearchValue >= initialValue) {
      stepSize /= 2.0; // restore and try again...
    }
    else {
      delete pc;
      return true; // found a better solution!
    }
  }

  delete pc;
  return true;
}

bool SQPFunctionMinimizer::printToFile(std::ofstream& file, unsigned int sqp_iter) {
  bool success = true;
  file << "[sqp_iteration] " << std::to_string(sqp_iter) << "\n";
  success &= qpProblem_.getQuadraticObjectiveFunctionPtr()->printToFile(file);
  success &= qpProblem_.getLinearFunctionConstraintsPtr()->printToFile(file);
  file << "[end_of_iter]\n";
  return success;
}

void SQPFunctionMinimizer::stop() {
  interrupt_ = true;
  qpSolver_->stop();
}

} // namespace numopt_sqp
