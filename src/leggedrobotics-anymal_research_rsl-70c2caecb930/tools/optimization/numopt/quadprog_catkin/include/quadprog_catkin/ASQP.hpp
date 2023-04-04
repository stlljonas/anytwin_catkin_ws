/*
 * ASQP.hpp
 *
 *  Created on: Mar 10, 2017
 *      Author: dbellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace asqp {

struct ActiveSet {
  Eigen::VectorXi A;
  Eigen::VectorXd xStar;
};

class ASQP {
 public:

  using SparseMat = Eigen::SparseMatrix<double, Eigen::RowMajor>;
  using CholeskySolver = Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper, Eigen::NaturalOrdering<int>>;

  using SparseMatrixRef = Eigen::Ref<SparseMat>;
  using SparseMatrixConstRef = const Eigen::Ref<const SparseMat>&;
  using MatrixRef = Eigen::MatrixXd&;
  using MatrixConstRef = const Eigen::MatrixXd&;

  using VectorRef = Eigen::VectorXd&;
  using VectorConstRef = const Eigen::VectorXd&;

/*! ASQP
 * @param maxIter             termination after max iteration is exceeded
 * @param isHessianConstant   true if Hessian remains constant over all nonlinear iterations
 */
  ASQP(int maxIter = 1000, bool isHessianConstant = false);
  virtual ~ASQP() = default;

  double minimize(SparseMat& G, const Eigen::VectorXd& g0, const SparseMat& CE,
                  const Eigen::VectorXd& ce0, const SparseMat& CI, const Eigen::VectorXd& ci0,
                  Eigen::VectorXd& x, asqp::ActiveSet* activeSet = nullptr, bool verbose = false,
                  int nonlinIter = 0);

  void printState() const;

  void stop();

 private:
  using ArrayXb = Eigen::Array<bool, Eigen::Dynamic, 1>;

//  static double solveUnconstrained(SparseMatrixConstRef G, VectorConstRef g0,
//                                   VectorRef x, CholeskySolver& choleskyG);

  static inline void compute_d(VectorRef d, MatrixConstRef J, VectorConstRef np);

  // Compute the step direction in the primal space.
  static inline void update_z(VectorRef z, MatrixConstRef J, VectorConstRef d, int iq);

  // Compute the negative of the step direction in the dual space.
  static inline void update_r(MatrixConstRef R, VectorRef r, VectorConstRef d, int iq);

  bool add_constraint(MatrixRef R, MatrixRef J, VectorRef d, int& iq, double& R_norm);

  void delete_constraint(MatrixRef R, MatrixRef J, int& iq, int l);

  static double distance(double a, double b);

  void step1();

  void computeStepLength();
  void computeStepDirection();

  double computePartialStepLength();
  inline double computeFullStepLength(double sp, VectorConstRef z, VectorConstRef np) const;

  // The number of optimization parameters.
  int paramCount_;

  // The number of equality constraints.
  int eqCount_;

  // The number of inequality constraints.
  int ineqCount_;

  // The number of iterations taken to solve the optimization problem.
  int iterations_;

  //! An instance of a sparse Cholesky solver.
  CholeskySolver choleskyG_;

  //! A definition of infinity.
  constexpr static double infinity_ = std::numeric_limits<double>::infinity();

  //! A vector of constraint deviations.
  Eigen::VectorXd constraintDeviation_;

  //! The indexes of the active constraints.
  Eigen::VectorXi activeSetIndexes_;

  //! The vector of Lagrangian multipliers.
  Eigen::VectorXd u_;

  //! Indexes of the active set.
  Eigen::VectorXi iai_;

  //! Flag for terminating the optimization.
  bool interrupt_;

  //! Optimization returns false if iter exceeds this number.
  int maxIter_;

  //! True if hessian remains constant over nonlin iterations.
  bool isHessianConstant_;

  //!
  Eigen::MatrixXd J0_;

  //! Estimation of the condition number of G.
  double condG_;
};

} /* namespace asqp */
