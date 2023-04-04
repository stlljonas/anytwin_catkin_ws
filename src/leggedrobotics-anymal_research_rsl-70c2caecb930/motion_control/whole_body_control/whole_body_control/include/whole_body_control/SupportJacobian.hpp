/*
 * SupportJacobian.hpp
 *
 *  Created on: Jan 27, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// stl
#include <memory>

namespace wbc {

class SupportJacobian {
 public:
  SupportJacobian() = default;
  virtual ~SupportJacobian() = default;

  SupportJacobian(SupportJacobian&&) = default;
  SupportJacobian(const SupportJacobian&) = default;

  SupportJacobian& operator=(SupportJacobian&&) = default;
  SupportJacobian& operator=(const SupportJacobian&) = default;

  /*! get the stacked support jacobian
   * @return      matrix with the Jacobian of all contact points
   */
  virtual const Eigen::MatrixXd& getSupportJacobianInForceFrame() const = 0;

  /*! get the transpose of stacked support jacobian
   * @return      matrix with the transpose of the Jacobian of all contact points
   */
  virtual const Eigen::MatrixXd& getSupportJacobianTransposeInForceFrame() const = 0;

  /*! get the stacked support jacobian derivation
   * @return      matrix with the Jacobian derivation of all contact points
   */
  virtual const Eigen::MatrixXd& getSupportJacobianTimeDerivativeInForceFrame() const = 0;

 protected:
  virtual bool updateSupport() = 0;
};

using SupportJacobianPtr = std::unique_ptr<SupportJacobian>;

} /* namespace wbc */
