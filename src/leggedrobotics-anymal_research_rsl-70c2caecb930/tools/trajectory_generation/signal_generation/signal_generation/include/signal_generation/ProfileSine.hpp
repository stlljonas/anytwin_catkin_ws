#pragma once


// signal generation
#include "signal_generation/ProfileBase.hpp"


namespace signal_generation {


template <typename PrimT_>
class ProfileSine : public ProfileBase<PrimT_>
{
protected:
  using Base = ProfileBase<PrimT_>;
  using PrimT = typename Base::PrimT;
  using RangeT = typename Base::RangeT;

  // Profile parameters.
  // Constant offset of the sine.
  const PrimT offset_ = 0.0;
  // Amplitude of the sine.
  const PrimT amplitude_ = 0.0;
  // Frequency of the sine.
  const PrimT frequency_ = 0.0;
  // Phase offset of the sine, relative to the start of the profile.
  const PrimT phaseOffset_ = 0.0;

  // Helper variables.
  const PrimT angularFrequency_ = 0.0;
  const PrimT amplitudeTimesAngularFrequency_ = 0.0;
  const PrimT amplitudeTimesAngularFrequencySquared_ = 0.0;

public:
  /*!
   * Constructor.
   * @param range       Range of the profile.
   * @param offset      Constant offset of the sine.
   * @param amplitude   Amplitude of the sine.
   * @param frequency   Frequency of the sine.
   * @param phaseOffset Phase offset of the sine, relative to the start of the profile.
   */
  ProfileSine(const RangeT range, const PrimT offset, const PrimT amplitude,
              const PrimT frequency, const PrimT phaseOffset)
  : Base(range),
    offset_(offset),
    amplitude_(amplitude),
    frequency_(frequency),
    phaseOffset_(phaseOffset),
    angularFrequency_(2.0*M_PI*frequency_),
    amplitudeTimesAngularFrequency_(amplitude_*angularFrequency_),
    amplitudeTimesAngularFrequencySquared_(amplitudeTimesAngularFrequency_*angularFrequency_) {}

  /*!
   * Constructor.
   * @param range       Range of the profile.
   * @param amplitude   Amplitude of the sine.
   * @param frequency   Frequency of the sine.
   * @param phaseOffset Phase offset of the sine, relative to the start of the profile.
   */
  ProfileSine(const RangeT range, const PrimT amplitude, const PrimT frequency, const PrimT phaseOffset)
  : ProfileSine(range, 0.0, amplitude, frequency, phaseOffset) {}

  virtual ~ProfileSine() {}

protected:
  PrimT getValueInRange(const PrimT time) const
  {
    return amplitude_*std::sin(angularFrequency_*(time-this->range_.getStart()) + phaseOffset_) + offset_;
  }

  PrimT getFirstDerivativeInRange(const PrimT time) const
  {
    return amplitudeTimesAngularFrequency_*std::cos(angularFrequency_*(time-this->range_.getStart()) + phaseOffset_);
  }

  PrimT getSecondDerivativeInRange(const PrimT time) const
  {
    return -amplitudeTimesAngularFrequencySquared_*std::sin(angularFrequency_*(time-this->range_.getStart()) + phaseOffset_);
  }
};

template <typename PrimT_>
using ProfileSinePtr = std::shared_ptr<ProfileSine<PrimT_>>;

using ProfileSineF = ProfileSine<float>;
using ProfileSineD = ProfileSine<double>;
using ProfileSineFPtr = ProfileSinePtr<float>;
using ProfileSineDPtr = ProfileSinePtr<double>;


} // signal_generation
