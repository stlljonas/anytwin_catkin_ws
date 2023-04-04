/*
 * dJ_lf.hpp
 *
 *  Created on: Oct 16, 2015
 *      Author: dbellicoso
 */

#pragma once

#include <Eigen/Core>

namespace starleth_kinematics {

Eigen::MatrixXd getJacobianRotationWorldToBaseInWorldFrame(const Eigen::VectorXd& generalizedPositionEulerXyz);
Eigen::MatrixXd getJacobianRotationDerivativeWorldToBaseInWorldFrame(const Eigen::VectorXd& generalizedPositionEulerXyz,
                                                                     const Eigen::VectorXd& generalizedVelocitiesEulerXyz);

Eigen::MatrixXd getJacobianWorldToLFFootInWorldFrameFromRobotModel(const Eigen::VectorXd& generalizedPositionsEulerXyz);
Eigen::MatrixXd getJacobianDerivativeWorldToLFFootInWorldFrameFromRobotModel(const Eigen::VectorXd& generalizedPositionsEulerXyz,
                                                                             const Eigen::VectorXd& generalizedVelocitiesEulerXyz);

}  // namespace starleth_kinematics
