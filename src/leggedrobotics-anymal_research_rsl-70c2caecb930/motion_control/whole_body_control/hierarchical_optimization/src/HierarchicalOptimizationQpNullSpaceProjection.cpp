/*
 * HierarchicalOptimizationQpNullSpaceProjection.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: Dario Bellicoso
 */

#include <hierarchical_optimization/HierarchicalOptimizationQpNullSpaceProjection.hpp>

// qp solver interface
#include <numopt_common/ParameterizationIdentity.hpp>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <boost/thread.hpp>

#include <message_logger/message_logger.hpp>

namespace hopt {

const Eigen::IOFormat eigenFormat(2, 0, ",", "\n", "[", "]");

HierarchicalOptimizationQpNullSpaceProjection::HierarchicalOptimizationQpNullSpaceProjection(
    int solutionDimension, std::unique_ptr<numopt_common::QuadraticProblemSolver>&& minimizer, bool useMultiThreading,
    double inequalityThreshold, double regularizer, internal::NullspaceBasisMethod nsMethod)
    : Base(solutionDimension),
      regularizer_(regularizer),
      minimizer_(std::move(minimizer)),
      quadraticProblem_(std::make_shared<numopt_common::QuadraticObjectiveFunction>(),
                        std::make_shared<numopt_common::LinearFunctionConstraints>()),
      cost_(0.0),
      useMultiThreading_(useMultiThreading),
      inequalityThreshold_(inequalityThreshold),
      nsMethod_(nsMethod) {
  resetOptimization();
}
HierarchicalOptimizationQpNullSpaceProjection::~HierarchicalOptimizationQpNullSpaceProjection() {
  if (nullspaceBaseThread_.joinable()) {
    nullspaceBaseThread_.join();
  }
}

// Eigen::MatrixXd PS(const Eigen::MatrixXd& mat) {
//  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectralDecomposition(mat);
//
//  Eigen::MatrixXd Q = spectralDecomposition.eigenvectors();
//  Eigen::VectorXd D = spectralDecomposition.eigenvalues();
//
//  for (unsigned int k=0; k<D.rows(); k++) {
//    MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] D before: " << std::endl <<
//    D.format(eigenFormat)); D(k) = std::max(static_cast<double>(D(k)), 0.0);
//    MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] D after: " << std::endl <<
//    D.format(eigenFormat));
//  }
//
//  Eigen::MatrixXd matPlus = Q*D.asDiagonal()*Q.transpose();
//
//  return matPlus;
//}
//
// Eigen::MatrixXd PU(const Eigen::MatrixXd& mat) {
//  Eigen::MatrixXd matOut = mat;
//
//  for (unsigned int k=0; k<mat.rows(); k++) {
//    matOut(k,k) = 1.0;
//  }
//
//  return matOut;
//}

bool HierarchicalOptimizationQpNullSpaceProjection::estimateNearestPositiveDefinite(const Eigen::MatrixXd& A, Eigen::MatrixXd& B) {
  if (A.size() == 0) {
    B = A;
    return true;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectralDecomposition(A);

  Eigen::MatrixXd Q = spectralDecomposition.eigenvectors();
  Eigen::VectorXd D = spectralDecomposition.eigenvalues();

  for (unsigned int k = 0; k < D.size(); k++) {
    D(k) = std::max(static_cast<double>(D(k)), regularizer_);
  }

  B = Q * D.asDiagonal() * Q.transpose();

  return true;
}

bool HierarchicalOptimizationQpNullSpaceProjection::solveOptimization(Eigen::VectorXd& solution) {
  bool success = true;

  // Stack the optimization problems according to their priority.
  buildPrioritizedOptimizationProblem();

  Eigen::MatrixXd inequalityJacobianStack(0, solutionDimension_);
  Eigen::VectorXd inequalityMaxValuesStack;

  // Nullspace projection matrix.
  Eigen::MatrixXd Zr = Eigen::MatrixXd::Identity(solutionDimension_, solutionDimension_);
  subProjection_ = Zr;

  // Solution vector.
  Eigen::VectorXd xStar = Eigen::VectorXd::Zero(solutionDimension_);

  auto lastTaskIt = std::prev(stackedTasks_.end());

  // Solve the hierarchically prioritized problems.
  for (auto it = stackedTasks_.begin(); it != stackedTasks_.end(); ++it) {
    const Eigen::MatrixXd& equalityConstraintJacobian = it->getEqualityConstraintJacobian();
    const Eigen::VectorXd& equalityConstraintTargetValues = it->getEqualityConstraintTargetValues();
    const Eigen::MatrixXd& inequalityConstraintJacobian = it->getInequalityConstraintJacobian();
    const Eigen::VectorXd& inequalityConstraintMaxValues = it->getInequalityConstraintMaxValues();

    const auto numEqConstr = equalityConstraintJacobian.rows();
    const auto numIneqConstr = inequalityConstraintJacobian.rows();

    // If Zr is the null vector, then there is no null space left in which to find a solution.
    const bool isFullyConstrained = Zr.isZero();
    const bool isLastTask = (it == lastTaskIt);
    const bool shouldComputeNullspace = (!isLastTask) && (numEqConstr > 0);

    unsigned int nullSpaceDimension = 0;
    if (!isFullyConstrained) {
      nullSpaceDimension = Zr.cols();
    }

    const auto qDimension = nullSpaceDimension + numIneqConstr;

    if (qDimension == 0) {
      //      MELO_WARN_THROTTLE_STREAM(1.0, "[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] When trying to solve
      //      problem named '"
      //                                << minProbName << "' a zero dimensional hessian was going to be constructed!");
      break;
    }

    /*
     * Each prioritized minimization problem can be written as:
     *
     *        min       ||Ax - b||^2 + mu*||v||^2_2 + sigma*||v||_1
     *        v,x
     *            s.to
     *                    C*x <= d + v
     *                      v >= 0
     *
     *  where v is a vector of slack variables. The 1-norm is used as an exact penalty function, and the 2-norm provides better conditioning
     * of the Hessian.
     *
     */

    /*
     * Assume that at stage p we have solved a set of constraints in the form:
     *
     *  A_{p}*x  = b_{p}
     *  D_{p}*x <= f_{p} + v_{p}
     *
     *  where v_{p} is a vector of slack variables for the problem at stage p.
     *  The current solution is then x*. If Zr is a matrix that projects into the nullspace of all equality constraint jacobians A_{k},
     *  for k = 1,...,p, then the solution for the next stage can be parametrized as:
     *
     *  x_{p+1} = x* + Zr*u_{p+1}
     *
     *  where u_{p+1} is a vector in the row space of Zr. Assuming the cost function is:
     *
     *  min 0.5*||Ax-b||^2 + 0.5*mu*||v||^2_2 + sigma*||v||_1
     *
     *  using the equation x = x* + Zr*u and assuming xi = [u; v] we can rewrite the cost function as:
     *
     *    0.5*( (Ax-b)' * (Ax-b) ) + 0.5*mu*(v' * v) + sigma*v =
     *
     *  = 0.5*( (A*x* + A*Zr*u - b)' * (A*x* + A*Zr*u - b) ) + 0.5*mu*(v' * v) + sigma*v   =      {use Ahat = A*Zr and bhat = A*x*-b}
     *
     *  = 0.5*(u'*Ahat'*Ahat*u) + (bhat'*Ahat*u) + 0.5*bhat'*bhat + 0.5*mu*(v' * v) + sigma*v  =      {use xi = [u; v]}
     *
     *  = 0.5*xi'*[Ahat'*Ahat     0]*xi  + [bhat'*Ahat 0]*xi + 0.5*(bhat'*bhat)
     *            [0           mu*I]       [sigma * ones]
     *
     *  Since bhat'*bhat does not influence the optimal solution, the cost function can finally be rewritten as:
     *
     *  min_{xi}  0.5*xi'*Q*xi + c'*x
     *
     *  where:
     *
     *    Q = [Ahat'*Ahat    0]
     *        [0          mu*I]
     *
     *    c = [Ahat'*bhat  ]
     *        [sigma * ones]
     *
     *  moreover, a regularizer is added to the diagonal of Q. This prevents zero eigenvalues when Ahat has a null-space.
     */
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(qDimension, qDimension);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(qDimension);

    // Set equality tasks
    if (numEqConstr > 0) {
      Eigen::MatrixXd Ahat = equalityConstraintJacobian * Zr;

      // If configured to do so, compute the nullspace basis in a separate thread.
      if (shouldComputeNullspace && useMultiThreading_) {
        nullspaceBaseThread_ = boost::thread(&HierarchicalOptimizationQpNullSpaceProjection::computeNullspaceBasis, this, Ahat, nsMethod_);
      }

      Q.topLeftCorner(nullSpaceDimension, nullSpaceDimension).noalias() = Ahat.transpose() * Ahat;
      c.topRows(nullSpaceDimension).noalias() = (Ahat.transpose() * (equalityConstraintJacobian * xStar - equalityConstraintTargetValues));
    }

    // Set weights for the slack variables
    if (numIneqConstr > 0) {
      /*
       * Weights on the hessian are set to the sum of eigenvalues of the equality tasks to prevent ill-conditioning.
       *    mu = trace(Ahat'*Ahat)
       * (In case no equality tasks exist, mu = 0)
       *
       * The linear weight is set based on the regularization parameter. The sqrt is used to convert from a regularization of
       * squared terms to linear terms:
       *    sigma = 1.0 / sqrt( regularizer )
       */
      if (nullSpaceDimension > 0) {
        Q.bottomRightCorner(numIneqConstr, numIneqConstr).diagonal().array() =
            Q.topLeftCorner(nullSpaceDimension, nullSpaceDimension).trace();
      }
      c.bottomRows(numIneqConstr).setConstant(1.0 / std::sqrt(regularizer_));
    }

    // Add a regularizer to the diagonal of Q.
    Q.diagonal().array() += regularizer_;

    quadraticProblem_.getQuadraticObjectiveFunctionPtr()->setGlobalHessian(Q.sparseView());
    quadraticProblem_.getQuadraticObjectiveFunctionPtr()->setLinearTerm(c);

    /*
     * At priority p inequality constraints are expressed by d_p <= D_p*x <= f_p
     * If at priority p the inequality jacobian is represented by the matrix D_p, then at iteration k, inequalityJacobianStack will be:
     *
     *  Dstack = [ D_1;
     *             D_2;
     *             ...
     *             D_k];
     *
     * while inequalityMaxValuesStack will be:
     *
     *  f_stack = [f_1;
     *             f_2;
     *             ...
     *             f_k];
     */

    /*
     * The set of inequality constraints can be rewritten as:
     *
     *  [D_{1}*Z_r     0] * xi_{p+1}  <=  - ( D_{1}   *  x* ) + f_{1} + v_{1}*
     *  ...
     *  [D_{p}*Z_r     0] * xi_{p+1}  <=  - ( D_{p}   *  x* ) + f_{p} + v_{p}*
     *  [D_{p+1}*Z_r  -I] * xi_{p+1}  <=  - ( D_{p+1} *  x* ) + f_{p+1}
     *
     *   Additionally, constraints must be added on the current slack variables:
     *
     *   0 <= [0   I] xi_{p+1} <= inf
     *
     */

    // Add rows to the stack of inequality jacobians and append the current matrix.
    const auto numHigherPriorityIneqConstr = inequalityJacobianStack.rows();
    inequalityJacobianStack.conservativeResize(numHigherPriorityIneqConstr + numIneqConstr, Eigen::NoChange);
    inequalityJacobianStack.bottomRows(numIneqConstr) = inequalityConstraintJacobian;

    // Add rows to the stack of inequality max values and append the current vector.
    inequalityMaxValuesStack.conservativeResize(numHigherPriorityIneqConstr + numIneqConstr, Eigen::NoChange);
    inequalityMaxValuesStack.bottomRows(numIneqConstr) = inequalityConstraintMaxValues;

    const auto numIneqConstrStack = inequalityJacobianStack.rows();

    /* Set the whole problem inequality jacobian.
     * The full inequality constraint jacobian will contain as many rows as the stack of inequality
     * jacobians plus as many rows as the current inequality constraint jacobian to account for
     * constraints on the slack variables.
     */
    Eigen::MatrixXd Dhat = Eigen::MatrixXd::Zero(numIneqConstrStack + numIneqConstr, qDimension);

    // The set of current and old inequality constraints.
    Dhat.topLeftCorner(numIneqConstrStack, nullSpaceDimension) = inequalityJacobianStack * Zr;

    // Account for the current slack variables.
    Dhat.block(numIneqConstrStack - numIneqConstr, nullSpaceDimension, numIneqConstr, numIneqConstr) =
        (-Eigen::VectorXd::Ones(numIneqConstr)).asDiagonal();

    // Account for constraints on the current slack variables.
    Dhat.bottomRightCorner(numIneqConstr, numIneqConstr) = Eigen::MatrixXd::Identity(numIneqConstr, numIneqConstr);

    // Set the whole problem inequality max values.
    Eigen::VectorXd fhat(Dhat.rows());
    fhat.head(numIneqConstrStack) = inequalityMaxValuesStack - inequalityJacobianStack * xStar;
    // Prevent that accumulated numerical errors tighten constraints that are active
    fhat.head(numHigherPriorityIneqConstr) = fhat.head(numHigherPriorityIneqConstr).cwiseMax(0.0);
    fhat.tail(numIneqConstr).setConstant(std::numeric_limits<double>::max());

    // Set the whole problem inequality min values.
    Eigen::VectorXd dhat(Dhat.rows());
    dhat.head(numIneqConstrStack).setConstant(-std::numeric_limits<double>::max());
    dhat.tail(numIneqConstr).setZero();

    if (inequalityThreshold_ != 0.0) {
      const Eigen::VectorXd DhatSum = Dhat.cwiseAbs().rowwise().sum();
      Eigen::MatrixXd DhatReduced = Eigen::MatrixXd::Zero((DhatSum.array() > inequalityThreshold_).count(), qDimension);
      Eigen::VectorXd dhatReduced = Eigen::VectorXd::Zero(DhatReduced.rows());
      Eigen::VectorXd fhatReduced = Eigen::VectorXd::Zero(DhatReduced.rows());

      int l = 0;
      for (int k = 0; k < DhatSum.rows(); ++k) {
        if (DhatSum(k) > inequalityThreshold_) {
          DhatReduced.row(l) = Dhat.row(k);
          dhatReduced.row(l) = dhat.row(k);
          fhatReduced.row(l) = fhat.row(k);
          ++l;
        }
      }

      quadraticProblem_.getLinearFunctionConstraintsPtr()->setGlobalInequalityConstraintJacobian(DhatReduced.sparseView());
      quadraticProblem_.getLinearFunctionConstraintsPtr()->setInequalityConstraintMaxValues(fhatReduced);
      quadraticProblem_.getLinearFunctionConstraintsPtr()->setInequalityConstraintMinValues(dhatReduced);

    } else {
      quadraticProblem_.getLinearFunctionConstraintsPtr()->setGlobalInequalityConstraintJacobian(Dhat.sparseView());
      quadraticProblem_.getLinearFunctionConstraintsPtr()->setInequalityConstraintMaxValues(fhat);
      quadraticProblem_.getLinearFunctionConstraintsPtr()->setInequalityConstraintMinValues(dhat);
    }

    /*
     * The solution vector xi is composed by the nullspace parameters u and the slack variables v:
     *      xi' = [u' v']
     */
    numopt_common::ParameterizationIdentity xiStar(qDimension);

    // Solve the QP problem.
    if (!minimizer_->minimize(&quadraticProblem_, xiStar, cost_)) {
      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] Problem named '" << it->getName()
                                                                                                            << "' returned false!");
      checkConstraintFeasibility(qDimension, it->getName());
      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] Nullspace basis: " << Zr.format(eigenFormat));
      success = false;
    }

    if (!xiStar.getParams().allFinite()) {
      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] Problem named '" << it->getName()
                                                                                                            << "' yielded NAN!");
      success = false;
    }

    // Wait for nullspace basis computation to terminate.
    if (nullspaceBaseThread_.joinable()) {
      nullspaceBaseThread_.join();
    }

    if (!success) {
      //      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] name: " << it->getName());
      //      numopt_common::SparseMatrix Q;
      //      quadraticProblem_.getQuadraticObjectiveFunctionPtr()->getGlobalHessian(Q, xiStar);
      //      Eigen::VectorXd c;
      //      quadraticProblem_.getQuadraticObjectiveFunctionPtr()->getLinearTerm(c);
      //      Eigen::SparseMatrix<double, Eigen::RowMajor> jacobian;
      //      quadraticProblem_.getFunctionConstraintsPtr()->getGlobalInequalityConstraintJacobian(jacobian, xiStar);
      //      Eigen::VectorXd d;
      //      quadraticProblem_.getFunctionConstraintsPtr()->getInequalityConstraintMinValues(d);
      //      Eigen::VectorXd f;
      //      quadraticProblem_.getFunctionConstraintsPtr()->getInequalityConstraintMaxValues(f);
      //      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] Nullspace basis: " << std::endl <<
      //      Zr.format(eigenFormat)); MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] Q: " <<
      //      std::endl << Q.toDense().format(eigenFormat));
      //      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] c: " << std::endl <<
      //      c.transpose().format(eigenFormat)); MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] A: "
      //      << std::endl << equalityConstraintJacobian.format(eigenFormat));
      //      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] b: " << std::endl <<
      //      equalityConstraintTargetValues.transpose().format(eigenFormat));
      //      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] D: " << std::endl <<
      //      jacobian.toDense().format(eigenFormat)); MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization]
      //      d: " << std::endl << d.transpose().format(eigenFormat));
      //      MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] f: " << std::endl <<
      //      f.transpose().format(eigenFormat)); MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization]
      //      xStar: " << std::endl << (xiStar.getParams()).format(eigenFormat));
      return false;
    }

    // Update the current optimal solution.
    xStar += Zr * xiStar.getParams().head(nullSpaceDimension);
    //    MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::solveOptimization] Problem named " << it->getName() << " xStar:
    //    " << std::endl << xStar.transpose().format(eigenFormat));

    // Find the new projection to the null space of the higher priority equality constraint jacobians.
    if (shouldComputeNullspace) {
      if (!useMultiThreading_) {
        computeNullspaceBasis(equalityConstraintJacobian * Zr, nsMethod_);
      }
      Zr *= subProjection_;
    }

    // Update last ineq max value set with current optimal slack vector.
    if ((!isLastTask) && (numIneqConstr > 0)) {
      inequalityMaxValuesStack.bottomRows(numIneqConstr) += xiStar.getParams().tail(numIneqConstr);
    }
  }

  solution = xStar;

  return success;
}

bool HierarchicalOptimizationQpNullSpaceProjection::checkConstraintFeasibility(unsigned int qDimension, const std::string& taskName) {
  // Check the feasibility of the constraint set.
  numopt_common::QuadraticProblem quadraticProblem(std::make_shared<numopt_common::QuadraticObjectiveFunction>(),
                                                   std::make_shared<numopt_common::LinearFunctionConstraints>());
  quadraticProblem.getQuadraticObjectiveFunctionPtr()->setGlobalHessian(Eigen::MatrixXd::Identity(qDimension, qDimension).sparseView());
  quadraticProblem.getQuadraticObjectiveFunctionPtr()->setLinearTerm(Eigen::VectorXd::Zero(qDimension));

  quadraticProblem.getLinearFunctionConstraintsPtr()->setGlobalEqualityConstraintJacobian(
      quadraticProblem_.getLinearFunctionConstraintsPtr()->getGlobalEqualityConstraintJacobian());
  quadraticProblem.getLinearFunctionConstraintsPtr()->setEqualityConstraintTargetValues(
      quadraticProblem_.getLinearFunctionConstraintsPtr()->getEqualityConstraintTargetValues());

  quadraticProblem.getLinearFunctionConstraintsPtr()->setGlobalInequalityConstraintJacobian(
      quadraticProblem_.getLinearFunctionConstraintsPtr()->getGlobalInequalityConstraintJacobian());
  quadraticProblem.getLinearFunctionConstraintsPtr()->setInequalityConstraintMaxValues(
      quadraticProblem_.getLinearFunctionConstraintsPtr()->getInequalityConstraintMaxValues());
  quadraticProblem.getLinearFunctionConstraintsPtr()->setInequalityConstraintMinValues(
      quadraticProblem_.getLinearFunctionConstraintsPtr()->getInequalityConstraintMinValues());

  numopt_common::ParameterizationIdentity xiStarFeasibility(qDimension);
  double costFeasibility = 0.0;
  if (!minimizer_->minimize(&quadraticProblem, xiStarFeasibility, costFeasibility)) {
    // Constraint set is infeasible. Replace conflicting inequalities with equality constraints.
    MELO_WARN_STREAM("[HierarchicalOptimizationQpNullSpaceProjection::checkConstraintFeasibility] Problem named '"
                     << taskName << "' had an infeasible set!");
    return false;
  } else {
    return true;
  }
}

void HierarchicalOptimizationQpNullSpaceProjection::computeNullspaceBasis(const Eigen::MatrixXd& mat,
                                                                          hopt::internal::NullspaceBasisMethod method) {
  switch (method) {
    case (hopt::internal::NullspaceBasisMethod::QR): {
      Eigen::FullPivHouseholderQR<Eigen::MatrixXd> matQr(mat.transpose());
      const int subProjectionDimension = mat.cols() - matQr.rank();
      if (subProjectionDimension == 0) {
        subProjection_ = Eigen::VectorXd::Zero(matQr.matrixQ().cols());
      } else {
        subProjection_ = matQr.matrixQ().rightCols(mat.cols() - matQr.rank());
      }
    } break;

    case (hopt::internal::NullspaceBasisMethod::LU): {
      subProjection_ = mat.fullPivLu().kernel();
    } break;

    case (hopt::internal::NullspaceBasisMethod::SVD): {
      Eigen::JacobiSVD<Eigen::MatrixXd> matSvd(mat, Eigen::ComputeThinU | Eigen::ComputeFullV);
      Eigen::MatrixXd matSvdV = matSvd.matrixV();
      const int subProjectionDimension = mat.cols() - matSvd.nonzeroSingularValues();
      if (subProjectionDimension == 0) {
        subProjection_ = Eigen::VectorXd::Zero(matSvd.matrixV().rows());
      } else {
        subProjection_ = matSvdV.rightCols(mat.cols() - matSvd.nonzeroSingularValues());
      }
    } break;

    default:
      throw std::runtime_error(
          "[HierarchicalOptimizationQpNullSpaceProjection::getNullspaceBasis] Unknown nullspace basis computation method.");
  }
}

} /* namespace hopt */
