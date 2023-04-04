/*!
 * @file     ProfileStep.hpp
 * @author   Christian Gehring
 * @date     April, 2015
 * @version  1.0
 * @ingroup  robot_utils
 * @brief
 */

#pragma once

#include "robot_utils/schedule/Profile.hpp"

namespace robot_utils {

template <typename Value_>
class ProfileStep : public Profile<Value_> {
 public:
  using Value = Value_;
  using Base = Profile<Value_>;

  ProfileStep(const Value& value, double duration) : Base(duration), value_(value) {}

  ~ProfileStep() override = default;

  Value getValue(double time) override { return value_; }

  Value getValueFirstDerivative(double time) override { return 0.0 * value_; }

 protected:
  Value value_;
};

}  // namespace robot_utils
