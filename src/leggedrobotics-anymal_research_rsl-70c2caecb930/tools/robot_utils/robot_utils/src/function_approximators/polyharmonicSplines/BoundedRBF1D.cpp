/*!
 * @file    BoundedRBF1D.cpp
 * @author  Christian Gehring
 * @date    Feb, 2013
 * @version 1.0
 * @ingroup robot_utils
 */

#include "robot_utils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace rbf {

void BoundedRBF1D::setRBFData(const std::vector<double>& xInput, const std::vector<double>& fInput) {
  Eigen::Map<const Eigen::VectorXd> xInputEigen(&xInput[0], xInput.size());
  Eigen::Map<const Eigen::VectorXd> fInputEigen(&fInput[0], fInput.size());
  setRBFData(xInputEigen, fInputEigen);
}

void BoundedRBF1D::setRBFData(const Eigen::VectorXd& xInput, const Eigen::VectorXd& fInput) {
  assert(xInput.size() == fInput.size());
  // we want: f(xInput) = fInput for all input points
  // set up the system that says that: A * w_ = fInput

  // additional constraints
  const int nAdditionalConstraints = 4;
  int n = static_cast<int>(xInput.size());

  // insert two points at the begin and the end to force the first derivative of the
  // start and end point to be zero, such that
  // x_0 = x0 		f(x_0) = f(x_0)
  // x_1 = x0 + dx   	f(x_1) = f(x_0)
  // x_2 = x0 + 2*dx	f(x_2) = f(x_0)
  // x_n+1 = x_n-1 		f(x_n+1) = f(x_n-1)
  // x_n = x_n-1 - dx 	f(x_n) = f(x_n-1)
  // x_n-1 = x_n-1 - 2*dx f(x_n-1) = f(x_n-1)

  const double dx = 0.0001;
  xInput_.resize(n + 4);
  fInput_.resize(n + 4);
  xInput_.segment(2, n) = xInput;
  fInput_.segment(2, n) = fInput;
  xInput_(0) = xInput(0);
  xInput_(1) = xInput(0) + dx;
  xInput_(2) = xInput(0) + 2 * dx;
  xInput_(n - 1 + 2) = xInput(n - 1) - 2 * dx;
  xInput_(n + 2) = xInput(n - 1) - dx;
  xInput_(n + 1 + 2) = xInput(n - 1);
  fInput_(0) = fInput(0);
  fInput_(1) = fInput(0);
  fInput_(n + 2) = fInput(n - 1);
  fInput_(n + 1 + 2) = fInput(n - 1);
  n = static_cast<int>(xInput_.size());

  //	std::cout << xInput << std::endl;
  //	std::cout << fInput << std::endl;

  w_.resize(n + nAdditionalConstraints);

  Eigen::MatrixXd A(n + nAdditionalConstraints, n + nAdditionalConstraints);
  A.setZero();

  Eigen::VectorXd b(n + nAdditionalConstraints);
  b.setZero();

  for (int i = 0; i < n; i++) {
    b[i] = fInput[i];
  }

  // initialize the A values - basis function contributions
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      A(i, j) = evaluateBasisFunction(xInput_[i], xInput_[j]);
    }
  }

  // in addition we want the sum of the lambdas (w_) to be 0
  for (int i = 0; i < n; i++) {
    A(i, n) = 1;
    A(i, n + 1) = xInput_[i];
    A(i, n + 2) = xInput_[i] * xInput_[i];
    A(i, n + 3) = xInput_[i] * xInput_[i] * xInput_[i];
    A(n, i) = 1;
    A(n + 1, i) = xInput_[i];
    A(n + 2, i) = xInput_[i] * xInput_[i];
    A(n + 2, i) = xInput_[i] * xInput_[i] * xInput_[i];
  }

  // now solve the system...
  w_ = A.colPivHouseholderQr().solve(b);

  isInitialized_ = true;
}

double BoundedRBF1D::evaluate(double x) const {
  assert(isInitialized_ == true);

  int n = xInput_.size();

  double result = 0;

  if (n <= 0) {
    return result;
  }

  // Evaluate interpolant at x - first the contributions of the basis functions
  for (int k = 0; k < n; k++) {
    result += w_[k] * evaluateBasisFunction(xInput_[k], x);
  }
  // and now add the polynomial term contribution
  result += 1 * w_[n] + x * w_[n + 1] + x * x * w_[n + 2] + x * x * x * w_[n + 3];

  return result;
}

double BoundedRBF1D::evaluateFirstDerivative(double x) const {
  assert(isInitialized_ == true);
  int n = xInput_.size();

  double result = 0;

  if (n <= 0) {
    return result;
  }

  // Evaluate interpolant at x - first the contributions of the basis functions
  for (int k = 0; k < n; k++) {
    result += w_[k] * dBFdx(xInput_[k], x);
  }
  // and now add the polynomial term contribution
  result += w_[n + 1] + 2 * x * w_[n + 2] + 3 * x * x * w_[n + 3];

  return result;
}

double BoundedRBF1D::evaluateSecondDerivative(double x) const {
  assert(isInitialized_ == true);
  int n = xInput_.size();
  double result = 0;

  if (n <= 0) {
    return result;
  }

  // Evaluate interpolant at x - first the contributions of the basis functions
  for (int k = 0; k < n; k++) {
    result += w_[k] * d2BFdx2(xInput_[k], x);
  }
  // and now add the polynomial term contribution
  result += 2 * w_[n + 2] + 6 * x * w_[n + 3];

  return result;
}

double BoundedRBF1D::evaluateThirdDerivative(double x) const {
  assert(isInitialized_ == true);
  int n = xInput_.size();
  double result = 0;

  if (n <= 0) {
    return result;
  }

  // Evaluate interpolant at x - first the contributions of the basis functions
  for (int k = 0; k < n; k++) {
    result += w_[k] * d3BFdx3(xInput_[k], x);
  }
  // and now add the polynomial term contribution
  result += 6 * w_[n + 3];

  return result;
}

}  // namespace rbf
