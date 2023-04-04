/*!
 * @file 	Stochastics.hpp
 * @author 	Christian Gehring
 % @date 	June 27, 2013
 * @version 1.0
 * @ingroup robot_utils
 *
 */

#pragma once

#include <Eigen/Core>

namespace robot_utils {

double myRan0(int* idum);
double myGasdev(int* idum);
double gaussian(double value, double std);

double normalPdf(const Eigen::VectorXd& value, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

template <unsigned int N_>
double normalPdf(const Eigen::Matrix<double, N_, 1>& value, const Eigen::Matrix<double, N_, 1>& mean,
                 const Eigen::Matrix<double, N_, N_>& cov);

double normalPdf(double value, double mean, double std);
double normalCdf(double value, double mean, double std);
double sampleNormal();
double sampleUniform();
double inverseNormal(double prob, double mean, double sd);

}  // namespace robot_utils

#include <robot_utils/math/Stochastics.tpp>
