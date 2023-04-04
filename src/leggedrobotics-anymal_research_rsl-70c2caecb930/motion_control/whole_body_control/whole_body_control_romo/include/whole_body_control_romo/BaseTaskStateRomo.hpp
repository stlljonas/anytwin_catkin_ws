/*
 * BaseTaskStateRomo.hpp
 *
 *  Created on: May 2, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// wholebody_romo
#include "whole_body_control_romo/typedefs.hpp"

// romo
#include <romo/RobotModel.hpp>

// loco
#include <loco/common/WholeBody.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class BaseTaskStateRomo {
 protected:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;

 public:
  BaseTaskStateRomo(const RobotModel& model, const loco::WholeBody& wholeBody);

  //! Base error getters
  const Eigen::Vector3d& getOrientationErrorInControlFrame() const { return orientationErrorInControlFrame_; }
  const loco::LocalAngularVelocity& getAngularVelocityErrorInControlFrame() const { return angularVelocityErrorInControlFrame_; }
  const loco::Position& getPositionErrorInControlFrame() const { return positionErrorInControlFrame_; }
  const loco::LinearVelocity& getLinearVelocityErrorInControlFrame() const { return linearVelocityErrorInControlFrame_; }

  //! Base jacobian getters
  const Eigen::MatrixXd& getJacobianTranslationWorldToTargetInControlFrame() const {
    return jacobianTranslationWorldToTargetInControlFrame_;
  }
  const Eigen::MatrixXd& getJacobianTranslationDerivativeWorldToTargetInControlFrame() const {
    return jacobianTranslationDerivativeWorldToTargetInControlFrame_;
  }
  const Eigen::MatrixXd& getJacobianRotationWorldToTargetInControlFrame() const {
    return jacobianRotationWorldToTargetInControlFrame_;
  }
  const Eigen::MatrixXd& getJacobianRotationDerivativeWorldToTargetInControlFrame() const {
    return jacobianRotationDerivativeWorldToTargetInControlFrame_;
  }

  bool advance(double dt);
  bool addVariablesToLog(const std::string& ns) const;

 private:
  bool computeErrors();
  bool computeTargetJacobians();

 protected:
  //! Robot model
  const RobotModel& model_;
  //! Whole body
  const loco::WholeBody& wholeBody_;

  //! motion control errors
  Eigen::Vector3d orientationErrorInControlFrame_;
  loco::LocalAngularVelocity angularVelocityErrorInControlFrame_;
  loco::Position positionErrorInControlFrame_;
  loco::LinearVelocity linearVelocityErrorInControlFrame_;

  //! Base target jacobians
  Eigen::MatrixXd jacobianSpatialWorldToTargetInWorldFrame_;
  Eigen::MatrixXd jacobianSpatialDerivativeWorldToTargetInWorldFrame_;
  //! Helpers
  Eigen::MatrixXd jacobianTranslationWorldToTargetInControlFrame_;
  Eigen::MatrixXd jacobianTranslationDerivativeWorldToTargetInControlFrame_;
  Eigen::MatrixXd jacobianRotationWorldToTargetInControlFrame_;
  Eigen::MatrixXd jacobianRotationDerivativeWorldToTargetInControlFrame_;
};

}  // namespace whole_body_control_romo

#include "whole_body_control_romo/BaseTaskStateRomo.tpp"
