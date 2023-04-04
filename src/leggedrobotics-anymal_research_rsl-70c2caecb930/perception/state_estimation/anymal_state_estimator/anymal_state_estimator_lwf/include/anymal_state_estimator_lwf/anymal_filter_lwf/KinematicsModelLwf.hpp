/*
 * KinematicsModelLwf.hpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// model
#include <anymal_model/AnymalModel.hpp>

namespace anymal_state_estimator_lwf {

// template <typename RobotModel_>
class KinematicsModelLwf : public anymal_model::AnymalModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit KinematicsModelLwf(double dt = 0.0025);
  ~KinematicsModelLwf() override = default;

  Eigen::Vector3d forwardKinematicsBaseToFootInBaseFrame(const Eigen::Vector3d& legJoints, int limbId);
  Eigen::Matrix3d getJacobianTranslationBaseToFootInBaseFrame(const Eigen::Vector3d& legJoints, int limbId);
};

} /* namespace anymal_state_estimator_lwf */
