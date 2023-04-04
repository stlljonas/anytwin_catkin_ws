#pragma once


// std
#include <deque>
#include <vector>
#include <iostream>

// signal generation
#include "signal_generation/ProfileBase.hpp"


namespace signal_generation {


template <typename PrimT_>
class Signal
{
protected:
  using PrimT = PrimT_;
  using RangeT = Range<PrimT>;
  using ProfileBasePtrT = ProfileBasePtr<PrimT>;

  std::vector<ProfileBasePtrT> profiles_;
  RangeT unifiedRange_ = RangeT(
      std::numeric_limits<PrimT>::infinity(),
      -std::numeric_limits<PrimT>::infinity());
  RangeT intersectedRange_ = RangeT(
      -std::numeric_limits<PrimT>::infinity(),
      std::numeric_limits<PrimT>::infinity());

public:
  Signal() {}
  virtual ~Signal() {}

  void addProfile(const ProfileBasePtrT& profile)
  {
    profiles_.push_back(profile);
    unifiedRange_.unifyWith(profile->getRange());
    intersectedRange_.intersectWith(profile->getRange());
  }

  void clear()
  {
    profiles_.clear();
    unifiedRange_ = RangeT(
        std::numeric_limits<PrimT>::infinity(),
        -std::numeric_limits<PrimT>::infinity());
    intersectedRange_ = RangeT(
        std::numeric_limits<PrimT>::infinity(),
        -std::numeric_limits<PrimT>::infinity());
  }

  const RangeT& getUnifiedRange() const
  {
    return unifiedRange_;
  }

  const RangeT& getIntersectedRange() const
  {
    return intersectedRange_;
  }

  PrimT getValue(const PrimT time) const
  {
    PrimT value = 0.0;
    for (const auto& profile : profiles_) {
      value += profile->getValue(time);
    }
    return value;
  }

  PrimT getFirstDerivative(const PrimT time) const
  {
    PrimT value = 0.0;
    for (const auto& profile : profiles_) {
      value += profile->getFirstDerivative(time);
    }
    return value;
  }

  PrimT getSecondDerivative(const PrimT time) const
  {
    PrimT value = 0.0;
    for (const auto& profile : profiles_) {
      value += profile->getSecondDerivative(time);
    }
    return value;
  }

  std::deque<PrimT> sampleValue(const RangeT& range, const PrimT timeStep) const
  {
    std::deque<PrimT> sampling;
    for (PrimT time = range.getStart(); time < range.getEnd(); time += timeStep) {
      sampling.push_back(getValue(time));
    }
    return sampling;
  }

  std::deque<std::pair<PrimT, PrimT>> sampleTimeAndValue(const RangeT& range, const PrimT timeStep) const
  {
    std::deque<std::pair<PrimT, PrimT>> sampling;
    for (PrimT time = range.getStart(); time < range.getEnd(); time += timeStep) {
      sampling.push_back({time, getValue(time)});
    }
    return sampling;
  }

  std::deque<PrimT> sampleFirstDerivative(const RangeT& range, const PrimT timeStep) const
  {
    std::deque<PrimT> sampling;
    for (PrimT time = range.getStart(); time < range.getEnd(); time += timeStep) {
      sampling.push_back(getFirstDerivative(time));
    }
    return sampling;
  }

  std::deque<std::pair<PrimT, PrimT>> sampleTimeAndFirstDerivative(const RangeT& range, const PrimT timeStep) const
  {
    std::deque<std::pair<PrimT, PrimT>> sampling;
    for (PrimT time = range.getStart(); time < range.getEnd(); time += timeStep) {
      sampling.push_back({time, getFirstDerivative(time)});
    }
    return sampling;
  }

  std::deque<PrimT> sampleSecondDerivative(const RangeT& range, const PrimT timeStep) const
  {
    std::deque<PrimT> sampling;
    for (PrimT time = range.getStart(); time < range.getEnd(); time += timeStep) {
      sampling.push_back(getSecondDerivative(time));
    }
    return sampling;
  }

  std::deque<std::pair<PrimT, PrimT>> sampleTimeAndSecondDerivative(const RangeT& range, const PrimT timeStep) const
  {
    std::deque<std::pair<PrimT, PrimT>> sampling;
    for (PrimT time = range.getStart(); time < range.getEnd(); time += timeStep) {
      sampling.push_back({time, getSecondDerivative(time)});
    }
    return sampling;
  }
};

using SignalF = Signal<float>;
using SignalD = Signal<double>;


} // signal_generation
