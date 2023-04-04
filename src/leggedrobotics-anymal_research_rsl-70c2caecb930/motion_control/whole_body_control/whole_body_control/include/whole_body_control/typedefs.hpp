/*
 * typedefs.hpp
 *
 *  Created on: Apr 27, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

#include <Eigen/Core>

namespace whole_body_control {

template<typename MatrixType>
using EigenRef = Eigen::Ref<MatrixType>;

template<typename MatrixType>
using EigenConstRef = const Eigen::Ref<const MatrixType>&;

}
