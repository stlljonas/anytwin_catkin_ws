#pragma once


// signal generation
#include "signal_generation/ProfileBase.hpp"


namespace signal_generation {


template <typename PrimT_>
class ProfileConstant : public ProfileBase<PrimT_>
{
protected:
  using Base = ProfileBase<PrimT_>;
  using PrimT = typename Base::PrimT;
  using RangeT = typename Base::RangeT;

  // Profile parameters.
  // Constant value of the profile.
  const PrimT constant_ = 0.0;

public:
  /*!
   * Constructor.
   * @param range    Range of the profile.
   * @param constant Constant value of the profile.
   */
  ProfileConstant(const RangeT range, const PrimT constant)
  : Base(range),
    constant_(constant) {}

  virtual ~ProfileConstant() {}

protected:
  PrimT getValueInRange(const PrimT time) const
  {
    return constant_;
  }

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
using ProfileConstantPtr = std::shared_ptr<ProfileConstant<PrimT_>>;

using ProfileConstantF = ProfileConstant<float>;
using ProfileConstantD = ProfileConstant<double>;
using ProfileConstantFPtr = ProfileConstantPtr<float>;
using ProfileConstantDPtr = ProfileConstantPtr<double>;


} // signal_generation
