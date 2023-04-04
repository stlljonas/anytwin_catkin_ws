/*
 * eQuadProg++.hh
 *
 *  Created on: Jun 5, 2015
 *      Author: Dario Bellicoso
 *        Note: This file is adapted by Dario Bellicoso from QuadProg++ to support the Eigen library
 */

#pragma once

#include <Eigen/Core>

namespace quadprog_catkin {

double minimize(const Eigen::MatrixXd& G, const Eigen::VectorXd& g0, const Eigen::MatrixXd& CE,
                const Eigen::VectorXd& ce0, const Eigen::MatrixXd& CI, const Eigen::VectorXd& ci0,
                Eigen::VectorXd& x, bool verbose = false);

} /* namespace quadprog_catkin */
