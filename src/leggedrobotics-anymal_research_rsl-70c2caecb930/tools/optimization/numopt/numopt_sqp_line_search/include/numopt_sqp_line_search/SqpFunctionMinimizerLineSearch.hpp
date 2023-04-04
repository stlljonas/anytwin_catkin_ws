/*!
* @file    SqpFunctionMinimizerLineSearch.hpp
* @author  Fabian Jenelten
* @date    Mar 12, 2018
*/

#pragma once

// numopt common
#include <numopt_sqp/SQPFunctionMinimizer.hpp>

namespace numopt_sqp {

struct LineSearchOptions {
  //! Line Search options.
  double alpha_;
  double beta_;

  //! Merit function weights.
  double muObjective_;
  double muEqConstraints_;
  double muIneqConstraints_;

  //! If true, inequality weights are estimated.
  bool adaptEqualityWeight_;

  unsigned int maxLineSearchIterations_;

  double directionalDerivativeResidual_;

  LineSearchOptions() :
    alpha_(0.4),
    beta_(0.6),
    muObjective_(1.0),
    muEqConstraints_(1.0),
    muIneqConstraints_(1.0),
    adaptEqualityWeight_(false),
    maxLineSearchIterations_(10u),
    directionalDerivativeResidual_(0.0) {
  }
};

/**
  Use the Sequential Quadratic Programming method to optimize a function, subject to constraints.

  Task: Find p that minimize f(p), such that Ap = b and d <= Cp <= f
  This means df/dp(p) = 0. SQP works similar as Newton iteration, i.e. approximates the environment of f as
  f(p + dp) ~ f(p) + df/dp(p)*dp + 1/2 dp' * d/dp(df/dp)(p) * dp
  Which is minimized when df/dp is closest to zero within the constraints.

  SQP hence solves a sequence of QP problems of the form
    min d/dp(df/dp)*dp + df/dp, s.t. A(p + dp) = b and C(p + dp) <= f
  which gives us the stepping direction dp within the constraint manifold. Iterating the above will
  hopefully yield p that minimizes f.

  A warning: Convergence can be improved in some cases by doing line search in the direction of dp.
  However, this can give us intermediate points outside the constraint manifold, which is bad.
  If you're unsure whether your constraints are non-convex, set 'maxLineSearchIterations' to 0.
*/
class SqpFunctionMinimizerLineSearch: public numopt_sqp::SQPFunctionMinimizer {
 private:
  using Base = numopt_sqp::SQPFunctionMinimizer;

 public:
  /*!
   * @param maxIterations               maximum number of iterations
   * @param qpSolver                    Quadratic Program solver
   * @param solveResidual               abortion criterium
   * @param solveFunctionValue          abortion criterium (if function value is lower than this value)
   * @param printOutput                 print infomrations to terminal
   * @param checkConstraints            check satisfaction of equality and inequality constraints
   */
  explicit SqpFunctionMinimizerLineSearch(
      std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver,
      int maxIterations = 2000, double solveResidual = 0.0001,
      double solveFunctionValue = -DBL_MAX, bool printOutput = false,
      bool checkConstraints = false);

  ~SqpFunctionMinimizerLineSearch() override = default;

  /**
    min f(p) subject to the constraints...
  */
  bool minimize(numopt_common::ConstrainedNonlinearProblem* function,
                numopt_common::Parameterization& params,
                double &functionValue,
                const std::string& fileName = "",
                bool printProblemToFile = false) override;

  /*!
   * Set line search options. The line search finds an optimal step length "h" by minimizing a merit function "Phi(x)" along
   * the optimization direction "dp". The merit function is defined as
   *    Phi(x) = f(x) + ||c_eq||_1 + |c_ineq|_1
   * where f(x) is the objective, c_eq=0 the equality value vector and c_ineq>=0 the inequality constraint value vector.
   * The function ||.||_1 denotes the l1 norm and |g(x)|_1 the l1 norm for an augmented function
   *    bar g(x) = {g(x) if x>=0; 0 if x<0}.
   * The line search algorithm stops if the following criterion is satisfied
   *    Phi(x+h*dp) <= Phi(x) + h*alpha*D(Phi(x), p)
   * where D(Phi(x), p) denots the directional derivative of Phi(x) along "p" and the vector "p" is the current best solution.
   *
   * @param maxLineSearchIterations         if this number of line search iterations is achieved, the line search algorithm terminates.
   * @param alpha                           weight for the directional derivative with alpha in (0, 0.5)
   * @param beta                            after each iteration, the step size h is reduced by this amount with beta in (0,1)
   * @param mu_eqConstraints                weight for equality constraints (w.r.t. objective)
   * @param mu_ineqConstraints              weight for inequality constraints (w.r.t. objective)
   * @param adaptEqualityWeight             if true, weight for equality constraints a computed adaptively (Note: enable only if inequality constraints are not present)
   */
  void setLineSearchOptions(const LineSearchOptions& lineSearchOpt) noexcept;

  //! If true, computation time can be reduced by passing an appropriate initial guess to the minimize function.
  bool isExternalWarmStarted() const override;

private:
  /*! @returns alpha for computing next parameter set:  p(i+1) = p(i) + alpha*dp.
   *
   * @param function  objective function
   * @param p         current parameter set
   * @param dp        parameter variation
   * @param maxSteps  maximum number of steps the line search should do in worst case.
   *
   */
  bool doLineSearch(double& stepSize,
                    numopt_common::ConstrainedNonlinearProblem* const problem,
                    numopt_common::Parameterization& p,
                    const numopt_common::Delta& dp,
                    unsigned int sqp_iter) override;

protected:
  double computeMeritFunctionValue(
      numopt_common::NonlinearObjectiveFunction* const objectiveFunction,
      numopt_common::NonlinearFunctionConstraints* const constraints,
      numopt_common::Parameterization& p,
      unsigned int numOfBackTrace = 0u,
      double ineqConstraintAcceptance = 1e-8) const;

  double computeMeritFunctionDirectionalDerivative(
      numopt_common::NonlinearObjectiveFunction* const objectiveFunction,
      numopt_common::NonlinearFunctionConstraints* const constraints,
      const numopt_common::Vector& dp,
      const numopt_common::Parameterization& p,
      double ineqConstraintAcceptance = 1e-8) const;

  //! Return l1 norm of x by considering only elements with x(i)<0.
  double lp1NormMin0(
      const numopt_common::Vector& vector,
      double ineqConstraintAcceptance) const;

  //! Return l1 norm of x by considering only elements with x(i)<0 and activeSetId(i)=1
  double lp1NormMin0(
      const numopt_common::Vector& vector,
      double ineqConstraintAcceptance,
      const Eigen::VectorXi& activeSetId) const;

  // Indexes of active set.
  void computeActiveSet(
      numopt_common::NonlinearFunctionConstraints* const constraints,
      const numopt_common::Parameterization& p,
      double ineqConstraintAcceptance,
      Eigen::VectorXi& activeSet) const;

  //! Find equality constraints adaptively.
  void findEqualityWeight(
      numopt_common::NonlinearObjectiveFunction* const objectiveFunction,
      numopt_common::NonlinearFunctionConstraints* const constraints,
      const numopt_common::Vector& dp,
      const numopt_common::Parameterization& p);

  LineSearchOptions lineSearchOpt_;


};

} // namespace sooqp
