/*
 * value_undefined_traits.hpp
 *
 *  Created on: May 16, 2017
 *      Author: dbellicoso
 */

#pragma once

#include <limits>
#include <Eigen/Core>

namespace loco {
namespace value_undefined_traits {

template<typename ValueType_>
struct value_limits {
  using ValueType = ValueType_;
  static ValueType value_undefined() {
    return std::numeric_limits<ValueType>::quiet_NaN();
  }
};


template<>
struct value_limits<Eigen::Vector2d> {
  using ValueType = Eigen::Vector2d;
  static ValueType value_undefined() {
    return ValueType::Constant(std::numeric_limits<double>::quiet_NaN());
  }
};


template<>
struct value_limits<Eigen::Vector3d> {
  using ValueType = Eigen::Vector2d;
  static ValueType value_undefined() {
    return ValueType::Constant(std::numeric_limits<double>::quiet_NaN());
  }
};

} /* namespace value_undefined_traits */
} /* namespace loco */
