#pragma once


// signal generation
#include "signal_generation/ProfileBase.hpp"


namespace signal_generation {


template <typename PrimT_>
class ProfileRamp : public ProfileBase<PrimT_>
{
protected:
  using Base = ProfileBase<PrimT_>;
  using PrimT = typename Base::PrimT;
  using RangeT = typename Base::RangeT;

  // Profile parameters.
  // Start value of the ramp.
  const PrimT start_ = 0.0;

  // End value of the ramp.
  const PrimT end_ = 0.0;

  // Helper variables.
  const PrimT gradient_ = 0.0;

public:
  /*!
   * Constructor.
   * @param range  Range of the profile.
   * @param start  Start of the profile in the beginning of the ramp.
   * @param end    End of the profile in the end of the ramp.
   */
  ProfileRamp(const RangeT range, const PrimT start, const PrimT end)
  : Base(range),
    start_(start),
    end_(end),
    gradient_((end_-start_)/this->range_.getLength()) {}

  /*!
   * Constructor.
   * @param range  Range of the profile.
   * @param end    End of the profile in the end of the ramp.
   */
  ProfileRamp(const RangeT range, const PrimT end)
  : ProfileRamp(range, 0.0, end) {}

  virtual ~ProfileRamp() {}

protected:
  PrimT getValueInRange(const PrimT time) const
  {
    return start_ + gradient_*(time - this->range_.getStart());
  }

  PrimT getFirstDerivativeInRange(const PrimT time) const
  {
    return gradient_;
  }

  PrimT getSecondDerivativeInRange(const PrimT time) const
  {
    return 0.0;
  }
};

template <typename PrimT_>
using ProfileRampPtr = std::shared_ptr<ProfileRamp<PrimT_>>;

using ProfileRampF = ProfileRamp<float>;
using ProfileRampD = ProfileRamp<double>;
using ProfileRampFPtr = ProfileRampPtr<float>;
using ProfileRampDPtr = ProfileRampPtr<double>;


} // signal_generation
