#pragma once


// signal generation
#include "signal_generation/ProfileBase.hpp"


namespace signal_generation {


template <typename PrimT_>
class ProfileStep : public ProfileBase<PrimT_>
{
protected:
  using Base = ProfileBase<PrimT_>;
  using PrimT = typename Base::PrimT;
  using RangeT = typename Base::RangeT;

  // Profile parameters
  // Absolute time when the step happens.
  const PrimT timeAbs_ = 0.0;
  // Start value of the profile before the step.
  const PrimT start_ = 0.0;
  // End value of the profile at and after the step.
  const PrimT end_ = 0.0;

public:
  /*!
   * Constructor.
   * @param range  Range of the profile.
   * @param time   Time when the step happens, relative to the start of the profile.
   * @param start  Start value of the step.
   * @param end    End value of the step.
   */
  ProfileStep(const RangeT range, const PrimT timeRel,
              const PrimT start, const PrimT end)
  : Base(range),
    timeAbs_(this->getRange().getStart() + timeRel),
    start_(start),
    end_(end) {}

  /*!
   * Constructor.
   * @param range  Range of the profile.
   * @param time   Time when the step happens, relative to the start of the profile.
   * @param end    End value of the step.
   */
  ProfileStep(const RangeT range, const PrimT timeRel, const PrimT end)
  : ProfileStep(range, timeRel, 0.0, end) {}

  virtual ~ProfileStep() {}

protected:
  PrimT getValueInRange(const PrimT time) const
  {
    if (time < timeAbs_) {
      return start_;
    } else {
      return end_;
    }
  }

  PrimT getFirstDerivativeInRange(const PrimT time) const
  {
    return (time != timeAbs_) ? 0.0 : std::numeric_limits<PrimT>::infinity();
  }

  PrimT getSecondDerivativeInRange(const PrimT time) const
  {
    return (time != timeAbs_) ? 0.0 : std::numeric_limits<PrimT>::quiet_NaN();
  }
};

template <typename PrimT_>
using ProfileStepPtr = std::shared_ptr<ProfileStep<PrimT_>>;

using ProfileStepF = ProfileStep<float>;
using ProfileStepD = ProfileStep<double>;
using ProfileStepFPtr = ProfileStepPtr<float>;
using ProfileStepDPtr = ProfileStepPtr<double>;


} // signal_generation
