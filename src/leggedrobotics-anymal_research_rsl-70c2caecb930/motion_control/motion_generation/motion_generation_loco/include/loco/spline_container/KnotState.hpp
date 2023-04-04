/*
 * KnotState.hpp
 *
 *  Created on: May 16, 2017
 *      Author: dbellicoso
 */

#pragma once

#include "loco/spline_container/value_undefined_traits.hpp"

namespace loco {

template<typename ValueType_>
class KnotState {
 public:

  using ValueType = ValueType_;

  KnotState(double timeValue, const ValueType& value)
      : timeValue_(timeValue),
        value_(value),
        valueFirstDerivative_(value_undefined_traits::value_limits<ValueType>::value_undefined()),
        valueSecondDerivative_(value_undefined_traits::value_limits<ValueType>::value_undefined())
  {

  }

  KnotState(double timeValue,
            const ValueType& value,
            const ValueType& valueFirstDerivative)
      : timeValue_(timeValue),
        value_(value),
        valueFirstDerivative_(valueFirstDerivative),
        valueSecondDerivative_(value_undefined_traits::value_limits<ValueType>::value_undefined())
  {

  }

  KnotState(double timeValue,
            const ValueType& value,
            const ValueType& valueFirstDerivative,
            const ValueType& valueSecondDerivative)
      : timeValue_(timeValue),
        value_(value),
        valueFirstDerivative_(valueFirstDerivative),
        valueSecondDerivative_(valueSecondDerivative)
  {

  }

  virtual ~KnotState() {

  }

  const ValueType& getTimeValue() const { return timeValue_; }
  const ValueType& getValue() const { return value_; }
  const ValueType& getValueFirstDerivative() const { return valueFirstDerivative_; }
  const ValueType& getValueSecondDerivative() const { return valueSecondDerivative_; }

  inline const ValueType getUndefinedValue() const {
    return value_undefined_traits::value_limits<ValueType>::value_undefined();
  }

 protected:
  double timeValue_;
  ValueType value_;
  ValueType valueFirstDerivative_;
  ValueType valueSecondDerivative_;
};


// Type definitions.
using KnotStateScalar = KnotState<double>;
using KnotStateVector = KnotState<Eigen::Vector3d>;

} /* namespace loco */
