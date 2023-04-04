#pragma once


// std
#include <cmath>
#include <memory>

// signal generation
#include "signal_generation/Range.hpp"


namespace signal_generation {


template <typename PrimT_>
class ProfileBase
{
protected:
  using PrimT = PrimT_;
  using RangeT = Range<PrimT>;

  const RangeT range_;
  const PrimT outOfRangeValue_ = 0.0;

public:
  ProfileBase(const RangeT range = RangeT())
  : range_(range) {}

  virtual ~ProfileBase() {}

  const RangeT& getRange() const
  {
    return range_;
  }

  PrimT getValue(const PrimT time) const
  {
    if (!range_.contains(time)) {
      return outOfRangeValue_;
    }
    return getValueInRange(time);
  }

  PrimT getFirstDerivative(const PrimT time) const
  {
    if (!range_.contains(time)) {
      return outOfRangeValue_;
    }
    return getFirstDerivativeInRange(time);
  }

  PrimT getSecondDerivative(const PrimT time) const
  {
    if (!range_.contains(time)) {
      return outOfRangeValue_;
    }
    return getSecondDerivativeInRange(time);
  }

protected:
  virtual PrimT getValueInRange(const PrimT time) const = 0;

  virtual PrimT getFirstDerivativeInRange(const PrimT time) const
  {
    return std::numeric_limits<PrimT>::quiet_NaN();
  }

  virtual PrimT getSecondDerivativeInRange(const PrimT time) const
  {
    return std::numeric_limits<PrimT>::quiet_NaN();
  }
};

template <typename PrimT_>
using ProfileBasePtr = std::shared_ptr<ProfileBase<PrimT_>>;


} // signal_generation
