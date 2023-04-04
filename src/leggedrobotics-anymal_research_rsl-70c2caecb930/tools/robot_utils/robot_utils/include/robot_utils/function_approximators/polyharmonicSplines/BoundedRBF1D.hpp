/*!
 * @file    BoundedRBF1D.hpp
 * @author  Christian Gehring
 * @date    Feb, 2013
 * @version 1.0
 * @ingroup robot_utils
 */
#pragma once
#include <Eigen/Core>
#include <vector>
#include "RBF1D.hpp"

namespace rbf {

//! Function approximation of a bounded function with radial basis functions (polyharmonic splines)
/*!
 * Compute function f(x), so that f(x_i) = f_i. The form of f is:
 * f = sum_i w_i * F_i(r) + a + b*x + c*x*x + d*x*x*x, where F_i are basis functions, one for each (x_i, f_i) pair.
 * The weights w_i, as well as the polynomial coefficients a, b, c and d are computed so that the function interpolates the input data,
 * whereby four additional points are inserted at begin and the end to ensure zero first derivative at the ends
 * The basis function is F_i(r_i) = r_i*r_i*log(r_i) with r_i = (x_i-x)*(x_i-x)
 *
 */
class BoundedRBF1D : public RBF1D {
 public:
  //! Constructor
  BoundedRBF1D() = default;

  //! Destructor
  ~BoundedRBF1D() override = default;

  /*! Compute function f(x), so that f(x_i) = f_i. The form of f is:
   *	f = sum_i w_i * F_i(r) + a + b*x + c*x*x + d*x*x*x, where F_i are basis functions, one for each (x_i, f_i) pair
   * 	the weights w_i, as well as the polynomial coefficients a, b, c and d are computed so that the function interpolates the input data
   * 	The basis function is F_i(r_i) = r_i*r_i*log(r_i) with r_i = (x_i-x)*(x_i-x)
   * @param xInput	the centers x_i
   * @param fInput	the function values f_i
   */
  void setRBFData(const std::vector<double>& xInput, const std::vector<double>& fInput) override;
  void setRBFData(const Eigen::VectorXd& xInput, const Eigen::VectorXd& fInput) override;

  /*! Evaluates the interpolation function at x, i.e. computes f(x)
   * If x < min(xInput) or x > max(xInput), x is wrapped to the range [min(xInput) max(xInput)).
   * @param x	value
   * @return	f(x)
   */
  double evaluate(double x) const override;

  /*! Evaluates the first derivative of the interpolation function at x, i.e. computes dfdx(x)
   * If x < min(xInput) or x > max(xInput), x is wrapped to the range [min(xInput) max(xInput)).
   * @param x	value
   * @return	 dfdx(x)
   */
  double evaluateFirstDerivative(double x) const override;

  /*! Evaluates the second derivative of the interpolation function at x, i.e. computes d2fdx2(x)
   * If x < min(xInput) or x > max(xInput), x is wrapped to the range [min(xInput) max(xInput)).
   * @param x	value
   * @return	 d2fdx2(x)
   */
  double evaluateSecondDerivative(double x) const override;

  /*! Evaluates the third derivative of the interpolation function at x, i.e. computes d3fdx3(x)
   * If x < min(xInput) or x > max(xInput), x is wrapped to the range [min(xInput) max(xInput)).
   * @param x	value
   * @return	 d3fdx3(x)
   */
  double evaluateThirdDerivative(double x) const override;
};

}  // namespace rbf
