/*!
 * @file     ProfileSinusoid.hpp
 * @author   Christian Gehring
 * @date     April, 2015
 * @version  1.0
 * @ingroup  robot_utils
 * @brief
 */

#pragma once

#include "kindr/Core"
#include "kindr/vectors/VectorBase.hpp"
#include "robot_utils/schedule/Profile.hpp"

namespace robot_utils {

namespace internal {
template <typename Profile_>
struct sinusoid_trait;
}  // namespace internal

template <typename Value_>
class ProfileSinusoid : public Profile<Value_> {
 public:
  using Value = Value_;
  using Base = Profile<Value_>;

  ProfileSinusoid(const Value& amplitude, const Value& offset, double frequency, double initialPhase, double duration)
      : Base(duration), amplitude_(amplitude), offset_(offset), frequency_(frequency), initialPhase_(initialPhase) {}

  ~ProfileSinusoid() override = default;

  virtual void setStartTime(double startTime) { this->startTime_ = startTime; }

  Value getValue(double time) override { return internal::sinusoid_trait<Value_>::getValue(*this, time); }

  Value getValueFirstDerivative(double time) override { return internal::sinusoid_trait<Value_>::getValueFirstDerivative(*this, time); }

  Value getValueSecondDerivative(double time) override { return internal::sinusoid_trait<Value_>::getValueSecondDerivative(*this, time); }

  const Value& getAmplitude() const { return amplitude_; }
  const Value& getOffset() const { return offset_; }
  double getFrequency() const { return frequency_; }

  double getInitialPhase() const { return initialPhase_; }

 protected:
  Value amplitude_;
  Value offset_;
  double frequency_;
  double initialPhase_;
};

namespace internal {
template <typename Value_>
struct sinusoid_trait {
  inline static Value_ getValue(const ProfileSinusoid<Value_>& profile, double time) {
    return (profile.getAmplitude() *
                sin(2.0 * M_PI * profile.getFrequency() * (time - profile.getStartTime()) + profile.getInitialPhase()) +
            profile.getOffset());
  }
  inline static Value_ getValueFirstDerivative(const ProfileSinusoid<Value_>& profile, double time) {
    return (2.0 * M_PI * profile.getFrequency() * profile.getAmplitude() *
            cos(2.0 * M_PI * profile.getFrequency() * (time - profile.getStartTime()) + profile.getInitialPhase()));
  }
  inline static Value_ getValueSecondDerivative(const ProfileSinusoid<Value_>& profile, double time) {
    return (-(2.0 * M_PI * profile.getFrequency()) * (2.0 * M_PI * profile.getFrequency()) * profile.getAmplitude() *
            sin(2.0 * M_PI * profile.getFrequency() * (time - profile.getStartTime()) + profile.getInitialPhase()));
  }
};

}  // namespace internal
}  // namespace robot_utils
