/*!
 * @file    PeriodicRBF1DC3.cpp
 * @author  Christian Gehring, Stelian Coros
 * @date    Feb, 2013
 * @version 1.0
 * @ingroup robot_utils
 */

#include "robot_utils/function_approximators/polyharmonicSplines/PeriodicRBF1DC3.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace rbf {

void PeriodicRBF1DC3::setRBFData(const std::vector<double>& xInput, const std::vector<double>& fInput) {
  Eigen::Map<const Eigen::VectorXd> xInputEigen(&xInput[0], xInput.size());
  Eigen::Map<const Eigen::VectorXd> fInputEigen(&fInput[0], fInput.size());
  setRBFData(xInputEigen, fInputEigen);
}

void PeriodicRBF1DC3::setRBFData(const Eigen::VectorXd& xInput, const Eigen::VectorXd& fInput) {
  assert(xInput.size() == fInput.size());
  assert(fInput[0] == fInput[fInput.size() - 1]);
  // we want: f(xInput) = fInput for all input points
  // sum w_i = 0 - somewhat arbitrary i guess - far away, only the polynomial part remains...
  // f'(x_0) = f'(x_{n-1}) - first derivative should be the same at the end points
  // f''(x_0) = f''(x_{n-1}) - second derivative should be the same at the end points
  // f'''(x_0) = f'''(x_{n-1}) - third derivative should be the same at the end points
  // set up the system that says that: A * w_ = fInput

  int n = static_cast<int>(xInput.size());
  xInput_ = xInput;
  fInput_ = fInput;
  w_.resize(xInput_.size() + 4);

  Eigen::MatrixXd A(n + 4, n + 4);
  A.setZero();

  Eigen::VectorXd b(n + 4);
  b.setZero();

  for (int i = 0; i < n; i++) {
    b[i] = fInput_[i];
  }

  // initialize the A values - basis function contributions
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      A(i, j) = evaluateBasisFunction(xInput[i], xInput[j]);
    }
  }

  // in addition we want the sum of the lambdas to be 0, and the
  // first and second derivatives to match at the end points x1 and x2 - also, add the polynomial contribution

  for (int i = 0; i < n; i++) {
    A(i, n) = 1;
    A(i, n + 1) = xInput_[i];
    A(i, n + 2) = xInput_[i] * xInput_[i];
    A(i, n + 3) = xInput_[i] * xInput_[i] * xInput_[i];
    A(n, i) = 1;
    A(n + 1, i) = dBFdx(xInput_[i], xInput_[0]) - dBFdx(xInput_[i], xInput_[n - 1]);
    A(n + 2, i) = d2BFdx2(xInput_[i], xInput_[0]) - d2BFdx2(xInput_[i], xInput_[n - 1]);
    A(n + 3, i) = d3BFdx3(xInput_[i], xInput_[0]) - d3BFdx3(xInput_[i], xInput_[n - 1]);
  }

  // first derivative
  A(n + 1, n + 2) = 2 * (xInput_[0] - xInput_[n - 1]);
  A(n + 1, n + 3) = 3 * (xInput_[0] * xInput_[0] - xInput_[n - 1] * xInput_[n - 1]);

  // second derivative
  A(n + 2, n + 3) = 6 * (xInput_[0] - xInput_[n - 1]);

  // now solve the system...
  w_ = A.colPivHouseholderQr().solve(b);

  isInitialized_ = true;
}

double PeriodicRBF1DC3::evaluate(double x) const {
  assert(isInitialized_ == true);

  int n = xInput_.size();
  x = wrapToRange(x, xInput_[0], xInput_[n - 1]);

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

double PeriodicRBF1DC3::evaluateFirstDerivative(double x) const {
  assert(isInitialized_ == true);
  int n = xInput_.size();
  x = wrapToRange(x, xInput_[0], xInput_[n - 1]);

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

double PeriodicRBF1DC3::evaluateSecondDerivative(double x) const {
  assert(isInitialized_ == true);
  int n = xInput_.size();
  x = wrapToRange(x, xInput_[0], xInput_[n - 1]);
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

double PeriodicRBF1DC3::evaluateThirdDerivative(double x) const {
  assert(isInitialized_ == true);
  int n = xInput_.size();
  x = wrapToRange(x, xInput_[0], xInput_[n - 1]);
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
