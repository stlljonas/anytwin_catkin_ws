/*
 * StarlethDynamics.hpp
 *
 *  Created on: Jul 25, 2016
 *      Author: dbellicoso
 */

#pragma once

// Eigen
#include <Eigen/Core>

namespace starleth_dynamics {
Eigen::VectorXd getGravityGeneralizedForces(const Eigen::VectorXd& eulerAnglesXyz);
Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd& eulerAnglesXyz);
Eigen::VectorXd getCoriolisGeneralizedForces(const Eigen::VectorXd& eulerAnglesXyz, const Eigen::VectorXd& eulerAnglesXyzDiff);
} /* namespace starleth_dynamics */
