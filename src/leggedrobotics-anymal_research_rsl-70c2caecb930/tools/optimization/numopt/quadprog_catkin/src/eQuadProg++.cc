/*
 * eQuadProg++.cc
 *
 *  Created on: Jun 5, 2015
 *      Author: Dario Bellicoso
 *        Note: This file is adapted by Dario Bellicoso from QuadProg++ to support the Eigen library
 */

// std library
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

//
#include <quadprog_catkin/eQuadProg++.hh>

// Eigen
#include <Eigen/Cholesky>

namespace quadprog_catkin {

// Utility functions for updating some data needed by the solution method
void compute_d(Eigen::VectorXd& d, const Eigen::MatrixXd& J, const Eigen::VectorXd& np);
void update_z(Eigen::VectorXd& z, const Eigen::MatrixXd& J, const Eigen::VectorXd& d, int iq);
void update_r(const Eigen::MatrixXd& R, Eigen::VectorXd& r, const Eigen::VectorXd& d, int iq);
bool add_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J, Eigen::VectorXd& d, int& iq, double& rnorm);
void delete_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J, Eigen::VectorXi& A, Eigen::VectorXd& u, int n, int p, int& iq, int l);

// Utility functions for computing the scalar product and the euclidean
// distance between two numbers
double distance(double a, double b);

typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;

// The Solving function, implementing the Goldfarb-Idnani method
double minimize(const Eigen::MatrixXd& G, const Eigen::VectorXd& linearTerm,
                      const Eigen::MatrixXd& CE, const Eigen::VectorXd& ce0,
                      const Eigen::MatrixXd& CI, const Eigen::VectorXd& ci0,
                      Eigen::VectorXd& x, bool verbose)
{

//  std::ostringstream msg;
//  {
//    //Ensure that the dimensions of the matrices and ublas::vectors can be
//    //safely converted from unsigned int into to int without overflow.
//    unsigned mx = std::numeric_limits<int>::max();
//    if(G.cols() >= mx || G.rows() >= mx ||
//       CE.rows() >= mx || CE.cols() >= mx ||
//       CI.rows() >= mx || CI.cols() >= mx ||
//       ci0.size() >= mx || ce0.size() >= mx || g0.size() >= mx){
//      msg << "The dimensions of one of the input matrices or ublas::vectors were "
//	  << "too large." << std::endl
//	  << "The maximum allowable size for inputs to solve_quadprog is:"
//	  << mx << std::endl;
//      throw std::logic_error(msg.str());
//    }
//  }
  const int n = G.cols();
  const int eqCount = CE.cols();
  const int ineqCount = CI.cols();
//  if ((int)G.rows() != n)
//  {
//    msg << "The cost hessian G is not a square matrix (" << G.rows() << " x "
//	<< G.cols() << ")";
//    throw std::logic_error(msg.str());
//  }
//  if (CE.cols() > 0 && (int)CE.rows() != n)
//  {
//    msg << "The equality constraints jacobian CE is incompatible (incorrect number of rows "
//	<< CE.rows() << " , expecting " << n << ")";
//    throw std::logic_error(msg.str());
//  }
//  if ((int)ce0.size() != p)
//  {
//    msg << "The equality constraints target vector ce0 is incompatible (incorrect dimension "
//	<< ce0.size() << ", expecting " << p << ")";
//    throw std::logic_error(msg.str());
//  }
//  if (CI.cols() > 0 && (int)CI.rows() != n)
//  {
//    msg << "The inequality constraints jacobian CI is incompatible (incorrect number of rows "
//	<< CI.rows() << " , expecting " << n << ")";
//    throw std::logic_error(msg.str());
//  }
//  if ((int)ci0.size() != m)
//  {
//    msg << "The inequality constraints jacobian ci0 is incompatible (incorrect dimension "
//	<< ci0.size() << ", expecting " << m << ")";
//    throw std::logic_error(msg.str());
//  }

  /*
   * n --> solution dimension
   * p --> number of equality constraints
   * m --> number of inequality constraints
   */

//  x.resize(n);
  int i, k, l; /* indices */
  int ip; // this is the index of the constraint to be added to the active set
  Eigen::VectorXd s(ineqCount + eqCount), z(n), r(ineqCount + eqCount), d(n), np(n), u(ineqCount + eqCount), x_old(n), u_old(ineqCount + eqCount);
  double psi, sum, ss, R_norm;
  const double inf = std::numeric_limits<double>::infinity();
  double t, t1, t2; /* t is the step length, which is the minimum of the partial step length t1
    * and the full step length t2 */
  Eigen::VectorXi A(ineqCount + eqCount), A_old(ineqCount + eqCount), iai(ineqCount + eqCount), iaexcl(ineqCount + eqCount);
  int q, iter = 0;
//  ArrayXb iaexcl(m + p);
  q = 0;  /* size of the active set A (containing the indices of the active constraints) */

  /*
   * Preprocessing phase
   */

  // Compute the trace of the original hessian G.
  const double c1 = G.trace();

  // Decompose G in the form L^T L
  Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> choleskyG;
  choleskyG.compute(G);

  if (choleskyG.info() != Eigen::ComputationInfo::Success ) {
    throw std::runtime_error("[quadprog_catkin::minimize] Cholesky decomposition was not successful");
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n,n);
  d.setZero();
  R_norm = 1.0;

  /* compute the inverse of the factorized ublas::matrix G^-1, this is the initial value for H */
  Eigen::MatrixXd J = choleskyG.matrixU().solve(Eigen::MatrixXd::Identity(n,n));
  const double c2 = J.trace();


  /* c1 * c2 is an estimate for cond(G) */

  /*
   * Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
   * this is a feasible point in the dual space
   * x = G^-1 * g0
   */
  x = -(choleskyG.solve(linearTerm));

  /* and compute the current solution value */
  double cost = 0.5 *linearTerm.dot(x);

  /* Add equality constraints to the working set A */
  int iq = 0;
  for (i = 0; i < eqCount; i++) {
//    if (CE.col(i).isZero()) continue;

    np = CE.col(i);
    compute_d(d, J, np);
    update_z(z, J, d, iq);
    update_r(R, r, d, iq);

    /* compute full step length t2: i.e., the minimum step in primal space s.t. the constraint
      becomes feasible */
    t2 = 0.0;

    if (!z.isZero()) {
      t2 = (-np.dot(x) - ce0(i)) / z.dot(np);
    }

    /* set x = x + t2 * z */
    x += t2 * z;

    /* set u = u+ */
    u(iq) = t2;
    u.head(iq) -= t2 * r.head(iq);

    /* compute the new solution value */
    cost += 0.5 * (t2 * t2) * z.dot(np);
    A(i) = -i - 1;

    if (!add_constraint(R, J, d, iq, R_norm))
    {
      // Equality constraints are linearly dependent
      throw std::runtime_error("Constraints are linearly dependent");
      return cost;
    }
  }

  /* set iai = K \ A */
  for (i = 0; i < ineqCount; i++)
    iai(i) = i;

l1:	iter++;
  /* step 1: choose a violated constraint */
  for (i = eqCount; i < iq; i++)
  {
    ip = A(i);
    iai(ip) = -1;
  }

  /* compute s(x) = ci^T * x + ci0 for all elements of K \ A */
  ss = 0.0;
  psi = 0.0; /* this value will contain the sum of all infeasibilities */
  ip = 0; /* ip will be the index of the chosen violated constraint */
  for (i = 0; i < ineqCount; i++)
  {
    iaexcl(i) = 1;
    sum = CI.col(i).dot(x) + ci0(i);
    s(i) = sum;
    psi += std::min(0.0, sum);
  }

  if (std::abs(psi) <= ineqCount * std::numeric_limits<double>::epsilon() * c1 * c2* 100.0)
  {
    /* numerically there are not infeasibilities anymore */
    q = iq;
    return cost;
  }

  /* save old values for u and A */
  u_old.head(iq) = u.head(iq);
  A_old.head(iq) = A.head(iq);

  /* and for x */
  x_old = x;

l2: /* Step 2: check for feasibility and determine a new S-pair */
    for (i = 0; i < ineqCount; i++)
    {
      if (s(i) < ss && iai(i) != -1 && iaexcl(i))
      {
        ss = s(i);
        ip = i;
      }
    }
  if (ss >= 0.0)
  {
    q = iq;
    return cost;
  }

  /* set np = n(ip) */
  np = CI.col(ip);
  /* set u = (u 0)^T */
  u(iq) = 0.0;
  /* add ip to the active set A */
  A(iq) = ip;

l2a:/* Step 2a: determine step direction */
  /* compute z = H np: the step direction in the primal space (through J, see the paper) */
  compute_d(d, J, np);
  update_z(z, J, d, iq);
  /* compute N* np (if q > 0): the negative of the step direction in the dual space */
  update_r(R, r, d, iq);
  /* Step 2b: compute step length */
  l = 0;
  /* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
  t1 = inf; /* +inf */
  /* find the index l s.t. it reaches the minimum of u+(x) / r */
  for (k = eqCount; k < iq; k++) {
    double tmp;
    if (r(k) > 0.0 && ((tmp = u(k) / r(k)) < t1) )
    {
      t1 = tmp;
      l = A(k);
    }
  }
  /* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
//  if (std::abs(z.dot(z)) > std::numeric_limits<double>::epsilon()) {
  if (!z.isZero()) {
    t2 = -s(ip) / z.dot(np);
  } else {
    t2 = inf; /* +inf */
  }

  /* the step is chosen as the minimum of t1 and t2 */
  t = std::min(t1, t2);
  /* Step 2c: determine new S-pair and take step: */

  /* case (i): no step in primal or dual space */
  if (t >= inf) {
    /* QPP is infeasible */
    // FIXME: unbounded to raise
    if (verbose) {
      std::cout << "[quadprog] QPP is unfeasible. No step in primal or dual space." << std::endl;
      std::cout << "[quadprog] Iterations: " << iter << std::endl;
    }
    q = iq;
    return inf;
  }
  /* case (ii): step in dual space */
  if (t2 >= inf)
  {
    /* set u = u +  t * (-r 1) and drop constraint l from the active set A */
    u.head(iq) -= t * r.head(iq);
    u(iq) += t;
    iai(l) = l;
    delete_constraint(R, J, A, u, n, eqCount, iq, l);
    goto l2a;
  }

  /* case (iii): step in primal and dual space */

  /* set x = x + t * z */
  x += t * z;
  /* update the solution value */
  cost += t * z.dot(np) * (0.5 * t + u(iq));
  u.head(iq) -= t * r.head(iq);
  u(iq) += t;

  if (std::abs(t - t2) < std::numeric_limits<double>::epsilon())
  {
    /* full step has taken */
    /* add constraint ip to the active set*/
    if (!add_constraint(R, J, d, iq, R_norm))
    {
      iaexcl(ip) = 0;
      delete_constraint(R, J, A, u, n, eqCount, iq, ip);
      for (i = 0; i < ineqCount; i++)
        iai(i) = i;
      for (i = eqCount; i < iq; i++)
	    {
	      A(i) = A_old(i);
	      u(i) = u_old(i);
				iai(A(i)) = -1;
	    }
      x = x_old;
      goto l2; /* go to step 2 */
    }
    else
      iai(ip) = -1;
    goto l1;
  }

  /* a partial step has taken */
  /* drop constraint l */
  iai(l) = l;
  delete_constraint(R, J, A, u, n, eqCount, iq, l);

  /* update s(ip) = CI * x + ci0 */
  s(ip) = CI.col(ip).dot(x) + ci0(ip);
  goto l2a;
}

inline void compute_d(Eigen::VectorXd& d, const Eigen::MatrixXd& J, const Eigen::VectorXd& np) {
  d = J.adjoint() * np;
}

inline void update_z(Eigen::VectorXd& z, const Eigen::MatrixXd& J, const Eigen::VectorXd& d, int iq) {
  z = J.rightCols(z.size()-iq) * d.tail(d.size()-iq);
}

inline void update_r(const Eigen::MatrixXd& R, Eigen::VectorXd& r, const Eigen::VectorXd& d, int iq) {
  r.head(iq)= R.topLeftCorner(iq,iq).triangularView<Eigen::Upper>().solve(d.head(iq));
}

bool add_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J, Eigen::VectorXd& d, int& iq, double& R_norm)
{
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
    if (std::abs(h) < std::numeric_limits<double>::epsilon()) {
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

  if (std::abs(d(iq - 1)) <= std::numeric_limits<double>::epsilon() * R_norm) {
    // problem degenerate
    return false;
  }
  R_norm = std::max<double>(R_norm, std::abs(d(iq - 1)));
  return true;
}

void delete_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J, Eigen::VectorXi& A, Eigen::VectorXd& u, int n, int p, int& iq, int l)
{
  int qq = -1;

  // Find the index qq for active constraint l to be removed.
  for (int i = p; i < iq; i++) {
    if (A(i) == l) {
      qq = i;
      break;
    }
  }

  // Remove the constraint from the active set and the duals.
  for (int i = qq; i < iq - 1; i++) {
    A(i) = A(i + 1);
    u(i) = u(i + 1);
    R.col(i) = R.col(i+1);
  }

  A(iq - 1) = A(iq);
  u(iq - 1) = u(iq);
  A(iq) = 0;
  u(iq) = 0.0;
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
    if (std::abs(h) < std::numeric_limits<double>::epsilon()) {
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

    for (int k = 0; k < n; k++) {
      const double t1 = J(k, j);
      const double t2 = J(k, j + 1);
      J(k, j) = t1 * cc + t2 * ss;
      J(k, j + 1) = xny * (J(k, j) + t1) - t2;
    }
  }
}

inline double distance(double a, double b) {
  const double a1{std::abs(a)};
  const double b1{std::abs(b)};
  if (a1 > b1)
  {
    const double t{(b1 / a1)};
    return a1 * std::sqrt(1.0 + t * t);
  }
  else
    if (b1 > a1)
    {
      const double t{(a1 / b1)};
      return b1 * std::sqrt(1.0 + t * t);
    }
  return a1 * std::sqrt(2.0);
}


} /* namespace quadprog_catkin */
