/*
 * NelderMeadOptimizer.hpp
 *
 *  Created on: Jul 19, 2015
 *      Author: Dario Bellicoso, Remo Diethelm
 */

#pragma once

// c++
#include <algorithm>
#include <iostream>
#include <functional>
#include <vector>

// eigen
#include <Eigen/Core>

namespace neldermead_optimizer {

namespace internal {

template <typename T>
static std::vector<int> getSortedIndexes(const std::vector<T>& v) {

  // Initialize original index locations.
  std::vector<int> idx(v.size());
  for (size_t k=0; k<idx.size(); k++) {
    idx[k] = k;
  }

  // Sort indexes based on comparing values in v.
  std::sort(idx.begin(), idx.end(), [&v](int i1, int i2) {return v[i1] <= v[i2];});

  return idx;
}

} /* namespace internal */

class NelderMeadOptimizer {
 private:
  using Fcn = std::function<double(const Eigen::VectorXd&)>;

 public:
  NelderMeadOptimizer();
  virtual ~NelderMeadOptimizer();

  bool optimize(double& fOpt,
                Eigen::VectorXd& xOpt,
                const Eigen::VectorXd& x0,
                const Fcn& fcn,
                bool minimize = true,
                long int maxIterations = -1,
                double maxFValueDiff = 1e-4,
                double maxVertexDiff = 1e-4);


 private:
  int getIterations();
  void setupSimplex();
  double evalFcn(const Eigen::VectorXd& x);
  void runMinimizationStep();
  bool terminateOptimization();

  void sortSimplex();
  void shrinkSimplex();
  void printSimplex() const;

  // Constants.
  const double rho_ = 1.0;
  const double chi_ = 2.0;
  const double psi_ = 0.5;
  const double sigma_ = 0.5;


  // Run time variables.
  long int numCoordinates_ = 0;
  long int numVertices_ = 0;
  long int s_ = 0;
  long int h_ = 0;

  Eigen::MatrixXd simplex_;
  std::vector<double> fValues_;

  unsigned int currentIteration_ = 0;

  // Optimization parameters.
  Eigen::VectorXd x0_;
  Fcn fcn_;
  bool minimize_ = true;
  unsigned int maxIterations_ = 0;
  double maxFValueDiff_ = 0.0;
  double maxVertexDiff_ = 0.0;

};

} /* namespace numopt_neldermead */
