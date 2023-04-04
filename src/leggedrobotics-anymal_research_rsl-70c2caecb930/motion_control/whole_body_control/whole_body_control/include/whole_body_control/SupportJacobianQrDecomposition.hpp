/*
 * SupportJacobian.hpp
 *
 *  Created on: Jan 27, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// whole body control
#include <whole_body_control/SupportJacobian.hpp>

// eigen
#include <Eigen/Core>


namespace wbc {

class SupportJacobianQrDecomposition : public SupportJacobian {
 public:
  SupportJacobianQrDecomposition() = default;
  ~SupportJacobianQrDecomposition() override = default;

  SupportJacobianQrDecomposition(SupportJacobianQrDecomposition&&) = default;
  SupportJacobianQrDecomposition(const SupportJacobianQrDecomposition&) = default;

  SupportJacobianQrDecomposition& operator=(SupportJacobianQrDecomposition&&) = default;
  SupportJacobianQrDecomposition& operator=(const SupportJacobianQrDecomposition&) = default;

  virtual const Eigen::MatrixXd& getP() const = 0;
  virtual const Eigen::MatrixXd& getQ() const = 0;
  virtual const Eigen::MatrixXd& getR() const = 0;
  virtual const Eigen::MatrixXd& getInvR() const = 0;
  virtual const Eigen::MatrixXd& getQu() const = 0;
  virtual const Eigen::MatrixXd& getQc() const = 0;
  virtual const Eigen::MatrixXd& getSu() const = 0;
  virtual const Eigen::MatrixXd& getSc() const = 0;
};

} /* namespace wbc */
