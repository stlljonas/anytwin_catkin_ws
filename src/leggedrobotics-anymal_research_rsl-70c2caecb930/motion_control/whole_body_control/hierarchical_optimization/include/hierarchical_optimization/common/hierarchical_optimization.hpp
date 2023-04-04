/*
 * hierarchical_optimization.hpp
 *
 *  Created on: Mar 14, 2017
 *      Author: dbellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hopt {

template<typename MatrixType>
using EigenRef = Eigen::Ref<MatrixType>;

template<typename MatrixType>
using EigenConstRef = const Eigen::Ref<const MatrixType>&;

}
