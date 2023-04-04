/*
 * StarlethDynamics.cpp
 *
 *  Created on: Jul 25, 2016
 *      Author: dbellicoso
 */

// Starleth Dynamics
#include "StarlethDynamics.hpp"

// Headers generate by Proneu for the model of StarlETH.
#include "CentrifugalCoriolisVector.hpp"
#include "GravityVector.hpp"
#include "MassMatrix.hpp"

namespace starleth_dynamics {

Eigen::VectorXd getGravityGeneralizedForces(const Eigen::VectorXd& eulerAnglesXyz) {
  return internal::getGravityGeneralizedForces(eulerAnglesXyz);
}

Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd& eulerAnglesXyz) {
  return internal::getMassMatrix(eulerAnglesXyz);
}

Eigen::VectorXd getCoriolisGeneralizedForces(const Eigen::VectorXd& eulerAnglesXyz, const Eigen::VectorXd& eulerAnglesXyzDiff) {
  return internal::getCoriolisGeneralizedForces(eulerAnglesXyz, eulerAnglesXyzDiff);
}

} /* namespace starleth_dynamics */
