/*
 * ASQP.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: dbellicoso
 */

// asqp
#include "quadprog_catkin/ASQP.hpp"

// stl
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

// eigen
#include <Eigen/SparseCholesky>
#include <Eigen/Cholesky>

#include<Eigen/IterativeLinearSolvers>

namespace asqp {

ASQP::ASQP(int maxIter, bool isHessianConstant)
    : paramCount_(0),
      eqCount_(0),
      ineqCount_(0),
      iterations_(0),
      choleskyG_(),
      constraintDeviation_(),
      activeSetIndexes_(),
      u_(),
      iai_(),
      interrupt_(false),
      maxIter_(maxIter),
      isHessianConstant_(isHessianConstant),
      J0_(),
      condG_(0.0)
{

}

double ASQP::minimize(SparseMat& G, const Eigen::VectorXd& g0, const SparseMat& CE,
                      const Eigen::VectorXd& ce0, const SparseMat& CI, const Eigen::VectorXd& ci0,
                      Eigen::VectorXd& x, asqp::ActiveSet* activeSet, bool verbose,
                      int nonlinIter) {
  if (interrupt_) { return infinity_; }

  iterations_ = 0;
  paramCount_ = G.rows();
  eqCount_ = CE.rows();
  ineqCount_ = CI.rows();

  int l = 0; /* indices */
  int ip = 0; // this is the index of the constraint to be added to the active set

  // Reset the vector of Lagrangian multipliers.
  u_.setZero(ineqCount_ + eqCount_);

  Eigen::VectorXd x_old(paramCount_), u_old(ineqCount_ + eqCount_);
  double psi, sum, ss;
  Eigen::VectorXi iaexcl(ineqCount_ + eqCount_);

  // Size of the active set A (containing the indices of the active constraints).
  int q = 0;

  // Compute the Cholesky decomposition of G = LU.
  if (!(isHessianConstant_ && nonlinIter > 0)) { // only compute if necessary
    choleskyG_.compute(Eigen::SparseMatrix<double, Eigen::ColMajor>(G));
    if (choleskyG_.info() != Eigen::ComputationInfo::Success) {
      throw std::runtime_error("[quadprog_catkin::minimize] Cholesky decomposition was not successful");
    }
  }
  if (interrupt_) { return infinity_; }

  // Solve the unconstrained problem. This is a feasible point in the dual space.
  if (activeSet != nullptr) {
    x = activeSet->xStar;
  } else {
    x = -(choleskyG_.solve(g0));
  }
  double cost = 0.5*(g0.dot(x));

  if (eqCount_+ineqCount_ == 0) {
    return cost;
  }

  // Initialize J with U^(-1).
  if (!(isHessianConstant_ && nonlinIter > 0)) { // only compute if necessary
    if (interrupt_) { return infinity_; }
    J0_ = (choleskyG_.matrixU().solve(Eigen::MatrixXd::Identity(paramCount_,paramCount_)));
    if (interrupt_) { return infinity_; }

    // Get the trace of the Hessian.
    double traceG = 0.0;
    for (auto k=0; k<G.rows(); k++) {
      traceG += G.coeffRef(k, k);
    }

    // Estimate the condition number of G.
    condG_ = traceG*J0_.trace();
  }

  Eigen::MatrixXd J = J0_;

  // Initialize the algorithm.
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(paramCount_,paramCount_);
  Eigen::VectorXd d = Eigen::VectorXd::Zero(paramCount_);
  constraintDeviation_ = Eigen::VectorXd::Zero(ineqCount_ + eqCount_);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(paramCount_);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(ineqCount_ + eqCount_);
  Eigen::VectorXd np = Eigen::VectorXd::Zero(paramCount_);
  activeSetIndexes_ = Eigen::VectorXi::Zero(ineqCount_ + eqCount_);
  Eigen::VectorXi A_old = Eigen::VectorXi::Zero(ineqCount_ + eqCount_);
  double R_norm = 1.0;
  int iq = 0;

  // Partial step length (maximum step in dual space without violating dual feasibility).
  double t1 = 0.0;

  // The full step length (minimum step in primal space such that the
  // p-th constraint becomes feasible).
  double t2 = 0.0;

  // Add the equality constraints to the working set A.
  for (int i = 0; i < eqCount_; i++) {
    if (interrupt_) { return infinity_; }
    // Get the normal np to the i-th constraint which will be added to the active set.
    np = CE.row(i);

    compute_d(d, J, np);
    update_z(z, J, d, iq);
    update_r(R, r, d, iq);

    t2 = 0.0;
    if (fabs(z.dot(z)) > std::numeric_limits<double>::epsilon()) {
      // The full step is compute as the negative ratio between the constraint deviation
      // and the dot product between the step direction in the primal space z and the
      // gradient of the newly added constraint np.
      t2 = -(np.dot(x) - ce0(i)) / z.dot(np);
    }

    /* set x = x + t2 * z */
    x += t2 * z;

    /* set u = u+ */
    u_(iq) = t2;
    u_.head(iq) -= t2 * r.head(iq);

    /* compute the new solution value */
    cost += 0.5 * (t2 * t2) * z.dot(np);
    activeSetIndexes_(i) = -i - 1;

    if (!add_constraint(R, J, d, iq, R_norm)) {
      // Equality constraints are linearly dependent.
      throw std::runtime_error("Constraints are linearly dependent");
      return cost;
    }
  }

  // Indexes of the Active set.
  iai_ = Eigen::VectorXi(ineqCount_ + eqCount_);

  /* set iai = K \ A */
  for (int i = 0; i < ineqCount_; i++) {
    iai_(i) = i;
  }

  if (activeSet != nullptr) {
    // Add the active inequality constraints to the working set.
    for (int k = 0; k <activeSet->A.size(); k++) {
      if (interrupt_) { return infinity_; }
      if (activeSet->A(k) == 1) {
        // Get the normal np to the i-th constraint which will be added to the active set.
        np = CI.row(k);

        compute_d(d, J, np);
        update_z(z, J, d, iq);
        update_r(R, r, d, iq);

        t2 = 0.0;
        if (fabs(z.dot(z)) > std::numeric_limits<double>::epsilon()) {
          // The full step is compute as the negative ratio between the constraint deviation
          // and the dot product between the step direction in the primal space z and the
          // gradient of the newly added constraint np.
          t2 = -(np.dot(x) - ci0(k)) / z.dot(np);
        }

        /* set x = x + t2 * z */
        x += t2 * z;

        /* set u = u+ */
        u_(iq) = t2;
        u_.head(iq) -= t2 * r.head(iq);

        /* compute the new solution value */
        cost += 0.5 * (t2 * t2) * z.dot(np);
        activeSetIndexes_(eqCount_+k) = -(eqCount_+k) - 1;

        if (!add_constraint(R, J, d, iq, R_norm)) {
          // Equality constraints are linearly dependent.
          throw std::runtime_error("Constraints are linearly dependent");
          return cost;
        }
      }
    }
  }

  // Step 1.
  l1: iterations_++;
  if (interrupt_) { return infinity_; }

  if (iterations_>maxIter_) {
    std::cout << "[ ASQP::minimize] Reached max number of iterations " <<  maxIter_ << std::endl;
    return infinity_;
  }

  /* step 1: choose a violated constraint */

  for (int i = eqCount_; i < iq; i++) {
    ip = activeSetIndexes_(i);
    iai_(ip) = -1;
  }

  /* compute s(x) = ci^T * x + ci0 for all elements of K \ A */
  ss = 0.0;
  psi = 0.0; /* this value will contain the sum of all infeasibilities */
  ip = 0; /* ip will be the index of the chosen violated constraint */
  for (int i = 0; i < ineqCount_; i++) {
    iaexcl(i) = 1;
    sum = -(CI.row(i).dot(x)) + ci0(i);
    constraintDeviation_(i) = sum;
    psi += std::min(0.0, sum);
  }

  if (fabs(psi) <= ineqCount_ * std::numeric_limits<double>::epsilon() * condG_ * 100.0) {
    /* numerically there are not infeasibilities anymore */
    q = iq;
    return cost;
  }

  /* save old values for u and A */
  u_old.head(iq) = u_.head(iq);
  A_old.head(iq) = activeSetIndexes_.head(iq);

  /* and for x */
  x_old = x;

  l2: /* Step 2: check for feasibility and determine a new S-pair */
  for (int i = 0; i < ineqCount_; i++) {
    if (constraintDeviation_(i) < ss && iai_(i) != -1 && iaexcl(i)) {
      ss = constraintDeviation_(i);
      ip = i;
    }
  }

  if (ss >= 0.0) {
    q = iq;
    return cost;
  }

  /* set np = paramCount_(ip) */
  np = -(CI.row(ip));
  /* set u = (u 0)^T */
  u_(iq) = 0.0;
  /* add ip to the active set A */
  activeSetIndexes_(iq) = ip;

  l2a:/* Step 2a: determine step direction */
    if (interrupt_) { return infinity_; }
    /* compute z = H np: the step direction in the primal space (through J, see the paper) */
    compute_d(d, J, np);
    update_z(z, J, d, iq);
    /* compute N* np (if q > 0): the negative of the step direction in the dual space */
    update_r(R, r, d, iq);
    /* Step 2b: compute step length */
    l = 0;
    /* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
    t1 = infinity_;
    /* find the index l s.t. it reaches the minimum of u+(x) / r */
    for (int k = eqCount_; k < iq; k++) {
      double tmp = 0.0;
      if (r(k) > 0.0 && ((tmp = u_(k) / r(k)) < t1) ) {
        t1 = tmp;
        l = activeSetIndexes_(k);
      }
    }
    /* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
    if (interrupt_) { return infinity_; }
    t2 = computeFullStepLength(constraintDeviation_(ip), z, np);
    if (interrupt_) { return infinity_; }

    /* the step is chosen as the minimum of t1 and t2 */
    const double t = std::min(t1, t2);
    /* Step 2c: determine new S-pair and take step: */

    /* case (i): no step in primal or dual space */
    if (t >= infinity_) {
      /* QPP is infeasible */
      // FIXME: unbounded to raise
      if (verbose) {
        std::cout << "[quadprog] QPP is unfeasible. No step in primal or dual space." << std::endl;
        std::cout << "[quadprog] Iterations: " << iterations_ << std::endl;
      }
      q = iq;
      return infinity_;
    }
    /* case (ii): step in dual space */
    if (t2 >= infinity_)
    {
      /* set u = u +  t * (-r 1) and drop constraint l from the active set A */
      u_.head(iq) -= t * r.head(iq);
      u_(iq) += t;
      iai_(l) = l;
      delete_constraint(R, J, iq, l);
      goto l2a;
    }

    /* case (iii): step in primal and dual space */

    /* set x = x + t * z */
    x += t * z;
    /* update the solution value */
    cost += t * z.dot(np) * (0.5 * t + u_(iq));
    u_.head(iq) -= t * r.head(iq);
    u_(iq) += t;

    if (fabs(t - t2) < std::numeric_limits<double>::epsilon()) {
      /* full step has taken */
      /* add constraint ip to the active set*/
      if (!add_constraint(R, J, d, iq, R_norm))
      {
        iaexcl(ip) = 0;
        delete_constraint(R, J, iq, ip);
        for (int i = 0; i < ineqCount_; i++) {
          iai_(i) = i;
        }
        for (int i = eqCount_; i < iq; i++) {
          activeSetIndexes_(i) = A_old(i);
          u_(i) = u_old(i);
          iai_(activeSetIndexes_(i)) = -1;
        }
        x = x_old;
        goto l2; /* go to step 2 */
      } else {
        iai_(ip) = -1;
      }
      goto l1;
    }

    /* a partial step has taken */
    /* drop constraint l */
    iai_(l) = l;
    if (interrupt_) { return infinity_; }
    delete_constraint(R, J, iq, l);

    /* update s(ip) = CI * x + ci0 */
    constraintDeviation_(ip) = -(CI.row(ip).dot(x)) + ci0(ip);
    goto l2a;

}

void ASQP::printState() const {
  std::cout << "State of the QP solver." << std::endl
            << "Active set indexes: " << std::endl << activeSetIndexes_.transpose() << std::endl
            << "iterations: " << iterations_ << std::endl
            << "iai: " << iai_.transpose() << std::endl;
}

double ASQP::computePartialStepLength() {

}

double ASQP::computeFullStepLength(double sp, VectorConstRef z, VectorConstRef np) const {
  if (fabs(z.dot(z)) > std::numeric_limits<double>::epsilon()) {
    return -sp / z.dot(np);
  } else {
    return infinity_;
  }
}

void ASQP::computeStepLength() {

}

void ASQP::computeStepDirection() {

}


void ASQP::step1() {
//  iter++;
//  /* step 1: choose a violated constraint */
//  for (int i = eqCount_; i < iq; i++) {
//    ip = A(i);
//    iai(ip) = -1;
//  }
//
//  /* compute s(x) = ci^T * x + ci0 for all elements of K \ A */
//  ss = 0.0;
//  psi = 0.0; /* this value will contain the sum of all infeasibilities */
//  ip = 0; /* ip will be the index of the chosen violated constraint */
//  for (int i = 0; i < ineqCount_; i++) {
//    iaexcl(i) = 1;
//    sum = -(CI.row(i).dot(x)) + ci0(i);
//    s(i) = sum;
//    psi += std::min(0.0, sum);
//  }
//
//  if (fabs(psi) <= ineqCount_ * std::numeric_limits<double>::epsilon() * condG * 100.0) {
//    /* numerically there are not infeasibilities anymore */
//    q = iq;
//    return cost;
//  }
//
//  /* save old values for u and A */
//  u_old.head(iq) = u.head(iq);
//  A_old.head(iq) = A.head(iq);
//
//  /* and for x */
//  x_old = x;

}

//double ASQP::solveUnconstrained(SparseMatrixConstRef G, VectorConstRef g0,
//                                VectorRef x, CholeskySolver& choleskyG) {
//  choleskyG.compute(Eigen::SparseMatrix<double, Eigen::ColMajor>(G));
//  if (choleskyG.info() != Eigen::ComputationInfo::Success ) {
//    throw std::runtime_error("[quadprog_catkin::minimize] Cholesky decomposition was not successful");
//  }
//
//  x = -(choleskyG.matrixU().solve(g0));
//  return 0.5 *g0.dot(x);
//}

void ASQP::compute_d(VectorRef d, MatrixConstRef J, VectorConstRef np) {
  // todo:: compare with J.transpose() * np;
  d = J.adjoint() * np;
}

void ASQP::update_z(VectorRef z, MatrixConstRef J, VectorConstRef d, int iq) {
  z = J.rightCols(z.size()-iq) * d.tail(d.size()-iq);
}

void ASQP::update_r(MatrixConstRef R, VectorRef r, VectorConstRef d, int iq) {
  r.head(iq)= R.topLeftCorner(iq,iq).triangularView<Eigen::Upper>().solve(d.head(iq));
}

bool ASQP::add_constraint(MatrixRef R, MatrixRef J, VectorRef d, int& iq, double& R_norm) {
  const int n = d.size();

  /* we have to find the Givens rotation which will reduce the element
    d(j) to zero.
    if it is already zero we don't have to do anything, except of
    decreasing j */
  for (int j = n - 1; j >= iq + 1; j--) {
    /* The Givens rotation is done with the ublas::matrix (cc cs, cs -cc).
    If cc is one, then element (j) of d is zero compared with element
    (j - 1). Hence we don't have to do anything.
    If cc is zero, then we just have to switch column (j) and column (j - 1)
    of J. Since we only switch columns in J, we have to be careful how we
    update d depending on the sign of gs.
    Otherwise we have to apply the Givens rotation to these columns.
    The i - 1 element of d has to be updated to h. */
    double cc = d(j - 1);
    double ss = d(j);
    const double h = distance(cc, ss);
    if (fabs(h) < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    d(j) = 0.0;
    ss /= h;
    cc /= h;
    if (cc < 0.0) {
      cc = -cc;
      ss = -ss;
      d(j - 1) = -h;
    } else {
      d(j - 1) = h;
    }
    const double xny = ss / (1.0 + cc);
    for (int k = 0; k < n; k++) {
      const double t1 = J(k, j - 1);
      const double t2 = J(k, j);
      J(k, j - 1) = t1 * cc + t2 * ss;
      J(k, j) = xny * (t1 + J(k, j - 1)) - t2;
    }
  }
  /* update the number of constraints added*/
  iq++;
  /* To update R we have to put the iq components of the d ublas::vector
    into column iq - 1 of R
    */

  R.col(iq-1).head(iq) = d.head(iq);

  if (fabs(d(iq - 1)) <= std::numeric_limits<double>::epsilon() * R_norm) {
    // Degenerate problem.
    return false;
  }
  R_norm = std::max<double>(R_norm, fabs(d(iq - 1)));
  return true;
}

void ASQP::delete_constraint(MatrixRef R, MatrixRef J, int& iq, int l) {
  int qq = -1;

  // Find the index qq for active constraint l to be removed.
  for (int i = eqCount_; i < iq; i++) {
    if (activeSetIndexes_(i) == l) {
      qq = i;
      break;
    }
  }

  // Remove the constraint from the active set and the duals.
  for (int i = qq; i < iq - 1; i++) {
    activeSetIndexes_(i) = activeSetIndexes_(i + 1);
    u_(i) = u_(i + 1);
    R.col(i) = R.col(i+1);
  }

  activeSetIndexes_(iq - 1) = activeSetIndexes_(iq);
  u_(iq - 1) = u_(iq);
  activeSetIndexes_(iq) = 0;
  u_(iq) = 0.0;
  for (int j = 0; j < iq; j++) {
    R(j, iq - 1) = 0.0;
  }
  /* constraint has been fully removed */
  iq--;

  if (iq == 0) {
    return;
  }

  for (int j = qq; j < iq; j++) {
    double cc = R(j, j);
    double ss = R(j + 1, j);
    const double h = distance(cc, ss);
    if (fabs(h) < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    cc /= h;
    ss /= h;
    R(j + 1, j) = 0.0;
    if (cc < 0.0) {
      R(j, j) = -h;
      cc = -cc;
      ss = -ss;
    } else {
      R(j, j) = h;
    }

    const double xny = ss / (1.0 + cc);
    for (int k = j + 1; k < iq; k++) {
      const double t1 = R(j, k);
      const double t2 = R(j + 1, k);
      R(j, k) = t1 * cc + t2 * ss;
      R(j + 1, k) = xny * (t1 + R(j, k)) - t2;
    }

    for (int k = 0; k < paramCount_; k++) {
      const double t1 = J(k, j);
      const double t2 = J(k, j + 1);
      J(k, j) = t1 * cc + t2 * ss;
      J(k, j + 1) = xny * (J(k, j) + t1) - t2;
    }
  }
}

double ASQP::distance(double a, double b) {
  const double a1 = fabs(a);
  const double b1 = fabs(b);

  if (a1 > b1) {
    const double t = (b1 / a1);
    return a1 * std::sqrt(1.0 + t * t);
  } else if (b1 > a1) {
    const double t = (a1 / b1);
    return b1 * std::sqrt(1.0 + t * t);
  }

  return a1 * std::sqrt(2.0);
}

void ASQP::stop() {
  interrupt_ = true;
}

} /* namespace asqp */
