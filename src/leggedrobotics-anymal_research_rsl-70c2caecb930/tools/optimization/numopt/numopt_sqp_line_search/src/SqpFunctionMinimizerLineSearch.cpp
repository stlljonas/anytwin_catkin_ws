/*!
* @file    SqpFunctionMinimizerLineSearch.cpp
* @author  Fabian Jenelten
* @date    Mar 12, 2018
*/

// numopt sqp
#include <numopt_sqp_line_search/SqpFunctionMinimizerLineSearch.hpp>

namespace numopt_sqp {

using namespace numopt_common;

SqpFunctionMinimizerLineSearch::SqpFunctionMinimizerLineSearch(
    std::shared_ptr<QuadraticProblemSolver> qpSolver,
    int maxIterations, double solveResidual,
    double solveFunctionValue, bool printOutput,
    bool checkConstraints)
  : Base(qpSolver, maxIterations, solveResidual,
         5, solveFunctionValue,
         printOutput, checkConstraints),
    lineSearchOpt_()
{ }

void SqpFunctionMinimizerLineSearch::setLineSearchOptions(const LineSearchOptions& lineSearchOpt) noexcept {
  lineSearchOpt_ = lineSearchOpt;
}

/**
  min f(p) subject to the constraints...
*/
bool SqpFunctionMinimizerLineSearch::minimize(
    ConstrainedNonlinearProblem* problem,
    Parameterization& params, double &functionValue,
    const std::string& fileName, bool printProblemToFile) {
  if (lineSearchOpt_.adaptEqualityWeight_) {
    lineSearchOpt_.muEqConstraints_ = -100000000.0;
  }

  return Base::minimize(problem, params, functionValue,
                 fileName, printProblemToFile);

}

bool SqpFunctionMinimizerLineSearch::doLineSearch(
    double& stepSize, ConstrainedNonlinearProblem* const problem,
    Parameterization& p, const Vector& dp, unsigned int sqp_iter)
{
  // By default, use unit step length.
  stepSize = 1.0;

  if (lineSearchOpt_.maxLineSearchIterations_ == 0u) { return true; }
  if (interrupt_) { return false; }

  // Initialization.
  std::unique_ptr<Parameterization> p_new(p.clone());

  // Update weight for equalities.
  findEqualityWeight(
      problem->getObjectiveFunctionPtr(),
      problem->getFunctionConstraintsPtr(),
      dp, p);

  // Compute merit function value at current SQP step p.
  const double merit_start = computeMeritFunctionValue(
      problem->getObjectiveFunctionPtr(),
      problem->getFunctionConstraintsPtr(),
      p);

  // Find backtracing step length based on the local gradient of the merit function along the direction dp.
  const double backTrackingLength = lineSearchOpt_.alpha_ *
      computeMeritFunctionDirectionalDerivative(
          problem->getObjectiveFunctionPtr(),
          problem->getFunctionConstraintsPtr(),
          dp, p);

  // If gradient along dp is too small, we do not expect a decrease in the merit function.
  if (std::fabs(backTrackingLength) < lineSearchOpt_.directionalDerivativeResidual_) {
    if (printOutput_) { std::cout << "[SqpFunctionMinimizerLineSearch::doLineSearch] Gradient is too small.\n"; }
    return false;
  }

  for (auto numOfBackTraces=1u; numOfBackTraces<=lineSearchOpt_.maxLineSearchIterations_; ++numOfBackTraces) {
    // Update solution vector.
    p_new->plus(p_new->getParams(), p.getParams(), static_cast<Vector>(stepSize*dp));
    if (optimizationStepInitCallback_) optimizationStepInitCallback_(*p_new);
    if (interrupt_) { return false; }

    // Compute new merit function value.
    const double merit_end = computeMeritFunctionValue(
        problem->getObjectiveFunctionPtr(),
        problem->getFunctionConstraintsPtr(),
        *p_new, numOfBackTraces);

    // Check if we have found optimal step length.
    if (merit_end <= merit_start + stepSize*backTrackingLength) {
      if (printOutput_) {
        std::cout << "[SqpFunctionMinimizerLineSearch::doLineSearch]. Found solution. Step length: " <<  stepSize << ", merit start" << merit_start << ", merit end = " << merit_end << ", back tracing = " << backTrackingLength <<  ".\n";
      }
      return true;
    }

    // Reduce step length.
    stepSize *= lineSearchOpt_.beta_;
  }

  if (printOutput_) {
    std::cout << "[SqpFunctionMinimizerLineSearch::doLineSearch] Num iter exceeded limit. Step length = " << stepSize <<  ".\n";
  }

  return false;
}

double SqpFunctionMinimizerLineSearch::computeMeritFunctionValue(
    NonlinearObjectiveFunction* const objectiveFunction,
    NonlinearFunctionConstraints* const constraints,
    Parameterization& p,
    unsigned int numOfBackTraces,
    double ineqConstraintAcceptance) const {
  /*
   * We minimize the merit function (Eq 17.22)
   *  Phi(x) = f(x) + ||c_eq||_1 + ||c_ineq||_1max0
   * where f(x) is the objective, c_eq=0 the equality value and c_ineq>=0 the inequality constraint value.
   * ||.||_1 denotes the l1 norm and ||x||_1max0 the l1 norm is defined as
   *    ||x||_1max0 = {
   *       0    if x>=0,
   *      -x    if x<0
   *    }
   */

  // Objective.
  double merit_objective = 0.0;
  if (lineSearchOpt_.muObjective_>0.0) {
    double objective;
    objectiveFunction->computeValue(objective, p);
    merit_objective = lineSearchOpt_.muObjective_*objective;
  }

  // Equality.
  double merit_equality = 0.0;
  if (constraints->getNumberOfEqualityConstraints()>0 && lineSearchOpt_.muEqConstraints_!=0.0) {
    Vector eqValue; Vector eqTargetValue;
    constraints->getEqualityConstraintValues(eqValue, p);
    constraints->getEqualityConstraintTargetValues(eqTargetValue);
    merit_equality = lineSearchOpt_.muEqConstraints_*(eqValue-eqTargetValue).eval().lpNorm<1>();
  }

  // Inequality.
  double merit_inequality = 0.0;
  if (constraints->getNumberOfInequalityConstraints()>0 && lineSearchOpt_.muIneqConstraints_>0.0) {
    Vector ineqValue; Vector ineqMinValue; Vector ineqMaxValue;
    constraints->getInequalityConstraintValues(ineqValue, p);
    constraints->getInequalityConstraintMinValues(ineqMinValue);
    constraints->getInequalityConstraintMaxValues(ineqMaxValue);

    // Combine inequality constraints -c(x) + upper >= 0 and c(x) - lower >= 0.
    Vector ineqTotValue(ineqValue.size()*2u);
    ineqTotValue << -ineqValue+ineqMaxValue,
                     ineqValue-ineqMinValue;

    merit_inequality = lineSearchOpt_.muIneqConstraints_*lp1NormMin0(ineqTotValue, ineqConstraintAcceptance);
  }

  if (printOutput_ && numOfBackTraces>0u) {
    std::cout << "[SqpFunctionMinimizerLineSearch::computeMeritFunctionValue] iter = " << numOfBackTraces <<
        ", objective = " << merit_objective <<
        ", eq = " << merit_equality <<
        ", ineq = " << merit_inequality <<
        ", tot = " << merit_objective + merit_equality + merit_inequality << std::endl;
  }

  return (merit_objective + merit_equality + merit_inequality);
}

double SqpFunctionMinimizerLineSearch::computeMeritFunctionDirectionalDerivative(
    NonlinearObjectiveFunction* const objectiveFunction,
    NonlinearFunctionConstraints* const constraints,
    const Vector& dp,
    const Parameterization& p,
    double ineqConstraintAcceptance) const {
  /*
   * "Numerical Optimization (2nd edition)", Nocedal and Wright
   * page 509, eq. 17.28.
   */
  double merit_value = 0.0;

  // Objective.
  if (lineSearchOpt_.muObjective_>0.0) {
    Vector gradient;
    objectiveFunction->getLocalGradient(gradient, p);
    merit_value += lineSearchOpt_.muObjective_*gradient.dot(dp); // Note: can be negative if we try to satisfy constraints
  }

  // Equality.
  if (constraints->getNumberOfEqualityConstraints()>0 && lineSearchOpt_.muEqConstraints_!=0.0) {
    SparseMatrix eqJacobian;
    constraints->getLocalEqualityConstraintJacobian(eqJacobian, p);
    merit_value += lineSearchOpt_.muEqConstraints_*(eqJacobian*dp).eval().lpNorm<1>();
  }

  // Inequality.
  if (constraints->getNumberOfInequalityConstraints()>0 && lineSearchOpt_.muIneqConstraints_>0.0) {
    SparseMatrix ineqJacobian;
    constraints->getLocalInequalityConstraintJacobian(ineqJacobian, p);

    // Combine inequality constraints -c(x) + upper >= 0 and c(x) - lower >= 0.
    Vector gradientValue(ineqJacobian.rows()*2u);
    const Vector temp = ineqJacobian*dp;
    gradientValue << -temp,
                      temp;

    Eigen::VectorXi activeSetId;
    computeActiveSet(constraints, p, ineqConstraintAcceptance, activeSetId);
    merit_value += lineSearchOpt_.muIneqConstraints_*(lp1NormMin0(gradientValue, ineqConstraintAcceptance, activeSetId));
  }

  return merit_value;
}

double SqpFunctionMinimizerLineSearch::lp1NormMin0(
    const Vector& vector,
    double ineqConstraintAcceptance) const {
  /*
   * Return the absolute l1 of all negative elements of vector.
   *
   * "Numerical Optimization (2nd edition)", Nocedal and Wright
   * page 507, eq 17.22.
   */

  double norm = 0.0;
  for (auto id=0u; id<vector.size(); ++id) {
    if (!std::isnan(vector(id)) && vector(id)<-ineqConstraintAcceptance) {
      norm -= vector(id);
    }
  }
  return norm;
}

double SqpFunctionMinimizerLineSearch::lp1NormMin0(
    const Vector& vector,
    double ineqConstraintAcceptance,
    const Eigen::VectorXi& activeSetId) const {
  /*
   * Return the absolute l1 of all negative elements of vector and are in the active set.
   *
   * "Numerical Optimization (2nd edition)", Nocedal and Wright
   * page 507, eq 17.22.
   */
  double norm = 0.0;
  for (auto id=0u; id<vector.size(); ++id) {
    if (!std::isnan(vector(id)) && vector(id)<-ineqConstraintAcceptance && activeSetId(id) == 1) {
      norm -= vector(id);
    }
  }
  return norm;
}

void SqpFunctionMinimizerLineSearch::computeActiveSet(
    NonlinearFunctionConstraints* const constraints,
    const Parameterization& p,
    double ineqConstraintAcceptance,
    Eigen::VectorXi& activeSet) const {
  /*
   * "Numerical Optimization (2nd edition)", Nocedal and Wright
   * page 323, Definition 12.1.
   *
   * c(x) = 0 -> constraint is active
   * c(x) > 0 -> constraint is not active
   */

  Vector ineqValue; Vector ineqMinValue; Vector ineqMaxValue;
  constraints->getInequalityConstraintValues(ineqValue, p);
  constraints->getInequalityConstraintMinValues(ineqMinValue);
  constraints->getInequalityConstraintMaxValues(ineqMaxValue);

  // Combine inequality constraints -c(x) + upper >= 0 and c(x) - lower >= 0.
  Vector ineqTotValue(ineqValue.size()*2u);
  ineqTotValue << -ineqValue+ineqMaxValue,
                   ineqValue-ineqMinValue;
  activeSet.resize(ineqTotValue.size());

  for (auto id=0u; id<ineqTotValue.size(); ++id) {
    if (std::fabs(ineqTotValue(id))<ineqConstraintAcceptance) { activeSet(id) = 1; } // active
    else { activeSet(id) = 0; } // not active
  }
}

void SqpFunctionMinimizerLineSearch::findEqualityWeight(
    NonlinearObjectiveFunction* const objectiveFunction,
    NonlinearFunctionConstraints* const constraints,
    const Vector& dp,
    const Parameterization& p) {

  /*
   * "Numerical Optimization (2nd edition)", Nocedal and Wright
   * page 542 eq. 18.36.
   * Find mu such that dp is a descent direction for the merit function.
   * Note: Computation valid only if no inequality constraints are present.
   */

  if (!lineSearchOpt_.adaptEqualityWeight_) { return; }
  if(constraints->getNumberOfInequalityConstraints()>0) {
    std::cout << "[SqpFunctionMinimizerLineSearch::findEqualityWeight] Problem contains inequality constraints. Adaptive equality weight cannot be computed!\n";
  }

  // Get local optimization data.
  Vector equalityValue; Vector equalityTargetValue;
  constraints->getEqualityConstraintValues(equalityValue, p);
  constraints->getEqualityConstraintTargetValues(equalityTargetValue);

  // Compute max equality error.
  const double equalityDiffNorm = (equalityValue-equalityTargetValue).lpNorm<1>();

  // Check if equality constraints are already satisfied.
 if (equalityDiffNorm > 1e-10) {
    Vector gradient;  SparseMatrix Hessian;
    objectiveFunction->getLocalGradient(gradient, p);
    objectiveFunction->getLocalHessian(Hessian, p);

    // Compute local optimal weight (rho = 0.5).
    const double mu_eqConstraints = ( gradient.dot(dp) + std::fmax(0.5*dp.dot(Hessian*dp), 0.0) ) / (0.5*equalityDiffNorm);

    // Update weight only if solution drifts away from the equality constraints.
    if (mu_eqConstraints > lineSearchOpt_.muEqConstraints_) {
      lineSearchOpt_.muEqConstraints_ = 1.01*mu_eqConstraints;
    }
  }
}

bool SqpFunctionMinimizerLineSearch::isExternalWarmStarted() const {
  return true;
}

} // namespace numopt_sqp
