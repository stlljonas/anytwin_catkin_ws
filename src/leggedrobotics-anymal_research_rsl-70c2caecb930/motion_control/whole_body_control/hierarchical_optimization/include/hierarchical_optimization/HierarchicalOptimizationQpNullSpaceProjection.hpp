/*
 * HierarchicalOptimizationQpNullSpaceProjection.hpp
 *
 *  Created on: Feb 11, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

#include <hierarchical_optimization/HierarchicalOptimizationBase.hpp>
#include <numopt_common/QuadraticProblemSolver.hpp>
#include <std_utils/std_utils.hpp>
#include <boost/thread.hpp>

#include <list>

namespace hopt {

namespace internal {
enum class NullspaceBasisMethod : unsigned int {
  QR = 0,
  SVD,
  LU
};
}

class HierarchicalOptimizationQpNullSpaceProjection final : public HierarchicalOptimizationBase {
 private:
  using Base = HierarchicalOptimizationBase;

 public:
  HierarchicalOptimizationQpNullSpaceProjection(int solutionDimension,
                                                std::unique_ptr<numopt_common::QuadraticProblemSolver>&& minimizer,
                                                bool useMultiThreading = false,
                                                double inequalityThreshold = 0.0,
                                                double regularizer = 1e-10,
                                                internal::NullspaceBasisMethod nsMethod = internal::NullspaceBasisMethod::LU);
  ~HierarchicalOptimizationQpNullSpaceProjection() override;

  bool solveOptimization(Eigen::VectorXd& solution) override;

 protected:
  bool estimateNearestPositiveDefinite(const Eigen::MatrixXd& A, Eigen::MatrixXd& B);
  void computeNullspaceBasis(const Eigen::MatrixXd& mat,
                             hopt::internal::NullspaceBasisMethod method = hopt::internal::NullspaceBasisMethod::QR);

  bool checkConstraintFeasibility(unsigned int qDimension, const std::string& taskName);

  //! A regularizer used to get a positive definite Hessian matrix.
  double regularizer_;

  // numerical solver
  std::unique_ptr<numopt_common::QuadraticProblemSolver> minimizer_;
  numopt_common::QuadraticProblem quadraticProblem_;
  double cost_;

  std_utils::HighResolutionClockTimer timer_;

  // Compute the nullspace basis in a separate thread.
  boost::thread nullspaceBaseThread_;
  Eigen::MatrixXd subProjection_;
  bool useMultiThreading_;
  double inequalityThreshold_;
  internal::NullspaceBasisMethod nsMethod_;
};

} /* namespace hopt */
