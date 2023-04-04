/*!
 * @file     ProfileLogChirpUpSweep.hpp
 * @author   Christian Gehring
 * @date     April, 2015
 * @version  1.0
 * @ingroup  robot_utils
 * @brief
 */

#pragma once

#include "robot_utils/function_generators/FctLogChirp.hpp"
#include "robot_utils/schedule/Profile.hpp"

namespace robot_utils {

template <typename Value_>
class ProfileLogChirpUpSweep : public Profile<double> {
 public:
  using Value = double;
  using Base = Profile<Value>;

  ProfileLogChirpUpSweep(const Value& offsetValue, const Value& maxValue, double minFrequencyHz, double maxFrequencyHz, double duration)
      : Base(duration), logChirp_() {
    logChirp_.setParamAmplitude(maxValue);
    logChirp_.setParamMinFrequencyHz(minFrequencyHz);
    logChirp_.setParamMaxFrequencyHz(maxFrequencyHz);
    logChirp_.setParamTimeInteval(duration);
    logChirp_.setParamOffset(offsetValue);
  }

  ~ProfileLogChirpUpSweep() override = default;

  Value getValue(double time) override {
    if (time < this->getStartTime()) {
      return logChirp_.getParamOffset();
    } else if (time > this->getEndTime()) {
      return logChirp_.getParamOffset();
    }
    return logChirp_.getUpSweepValue(time - this->getStartTime());
  }

  Value getValueFirstDerivative(double time) override { return logChirp_.getUpSweepFirstDerivativeValue(time - this->getStartTime()); }

 protected:
  FctLogChirp logChirp_;
};

}  // namespace robot_utils
