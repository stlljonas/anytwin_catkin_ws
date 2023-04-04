/*
This is similar than ProfileSine.hpp, but adapted to create a rectangular square waveform.
Mario Mauerer - Oct. 2018
*/

#pragma once


// signal generation
#include "signal_generation/ProfileBase.hpp"


namespace signal_generation {


template <typename PrimT_>
class ProfileSquare : public ProfileBase<PrimT_>
{
protected:
  using Base = ProfileBase<PrimT_>;
  using PrimT = typename Base::PrimT;
  using RangeT = typename Base::RangeT;

  // Profile parameters.
  // Constant offset of the square wave
  const PrimT offset_ = 0.0;
  // Amplitude of the square wave
  const PrimT amplitude_ = 0.0;
  // Period of the square wave
  const PrimT period_ = 0.0;
  // Duty cycle of the square wave
  PrimT duty_ = 0.5;

public:
  /*!
   * Constructor.
   * @param range       Range of the profile.
   * @param offset      Constant offset of the square wave
   * @param amplitude   Amplitude of the square wave.
   * @param frequency   Frequency of the square wave.
   * @param duty        Duty cycle of the waveform, in [0,1]
   */
  ProfileSquare(const RangeT range, const PrimT offset, const PrimT amplitude,
              const PrimT frequency, const PrimT duty)
  : Base(range),
    offset_(offset),
    amplitude_(amplitude),
    period_(1.0/frequency),
    duty_(duty) {
      // Check and limit the duty cycle into its range.
      if(duty > 1.0)
        duty_ = 1.0;
      else if(duty < 0.0)
        duty_ = 0.0;
    }

  /*!
   * Constructor, if no offset is given
   * @param range       Range of the profile.
   * @param amplitude   Amplitude of the square wave.
   * @param frequency   Frequency of the square wave.
   * @param duty        Duty cycle of the waveform, in [0,1]
   */
  ProfileSquare(const RangeT range, const PrimT amplitude, const PrimT frequency,
              const PrimT duty)
  : ProfileSquare(range, 0.0, amplitude, frequency, duty) {}

  virtual ~ProfileSquare() {}

protected:
  PrimT getValueInRange(const PrimT time) const
  {
    // Get the current time within a period:
    PrimT time_period = std::fmod(time,period_);
    if(time_period < (duty_ * period_)){
      return amplitude_ + offset_;
    }
    else{
      return -amplitude_ + offset_;
    }
  }

  // The derivatives are difficult/not really meaningful for a square wave.
  // The functions simply return zero to maintain compatibility.

  PrimT getFirstDerivativeInRange(const PrimT time) const
  {
    return 0.0;
  }

  PrimT getSecondDerivativeInRange(const PrimT time) const
  {
    return 0.0;
  }

};

template <typename PrimT_>
using ProfileSquarePtr = std::shared_ptr<ProfileSquare<PrimT_>>;

using ProfileSquareF = ProfileSquare<float>;
using ProfileSquareD = ProfileSquare<double>;
using ProfileSquareFPtr = ProfileSquarePtr<float>;
using ProfileSquareDPtr = ProfileSquarePtr<double>;


}
