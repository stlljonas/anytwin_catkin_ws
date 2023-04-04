#pragma once


// signal generation
#include "signal_generation/ProfileBase.hpp"


namespace signal_generation {


/*!
 * Reference of log chirp function:
 * A. Farina, "Simultaneous Measurement of Impulse Response and Distortion with a Swept-Sine Technique", AES, Paris, 2000.
 *
 * System Identification - rules TODO: move this to anydrive_tests
 * ------------------------------
 * Reference:
 * M. B. Tischler, R. K. Remple, "Aircraft and Rotorcraft System Identification - Engineering Methods with Flight Test Examples", 2006.
 *
 * Parameters of your system you need to know:
 * bandwidthFrequency_: the bandwidth of your system
 * outOfPhaseFrequency_: the frequency at which the phase of the response is exactly out of phase (-180°)
 *
 * Calculation of the required frequencies and time interval:
 * startFrequency_ = 0.5*bandwidthFrequency_;
 * endFrequency_ = 2.5*outOfPhaseFrequency_;
 * maxTimeInteval_ = 1.0/paramMinFrequency_;
 * range_.getLength() = 2.0*maxTimeInteval_;
 *
 * The experiment should start with two complete long-period inputs at the beginning of the sweep
 * corresponding to startFrequency_
 *
 * The total sweep record length should be about
 * recTimeInterval = (4 to 5)maxTimeInteval_
 */
template <typename PrimT_>
class ProfileLogChirpUpSweep : public ProfileBase<PrimT_>
{
protected:
  using Base = ProfileBase<PrimT_>;
  using PrimT = typename Base::PrimT;
  using RangeT = typename Base::RangeT;

  // Profile parameter.
  // Constant offset of the chirp.
  const PrimT offset_ = 0.0;
  // Amplitude of the chirp.
  const PrimT amplitude_ = 0.0;
  // Starting frequency of the chirp [Hz].
  const PrimT startFrequency_ = 0.0;
  // Ending frequency of the chirp [Hz].
  const PrimT endFrequency_ = 0.0;

  // Helper variables.
  const PrimT l_ = 0.0;
  const PrimT lInv_ = 0.0;
  const PrimT k_ = 0.0;

public:
  /*!
   * Constructor.
   * @param range          Range of the profile.
   * @param offset         Constant offset of the chirp.
   * @param amplitude      Amplitude of the chirp.
   * @param startFrequency Starting frequency of the chirp.
   * @param endFrequency   Ending frequency of the chirp.
   */
  ProfileLogChirpUpSweep(const RangeT range, const PrimT offset, const PrimT amplitude,
                         const PrimT startFrequency, const PrimT endFrequency)
  : Base(range),
    offset_(offset),
    amplitude_(amplitude),
    startFrequency_(startFrequency),
    endFrequency_(endFrequency),
    l_(this->range_.getLength()/std::log(endFrequency_/startFrequency_)),
    lInv_(1.0/l_),
    k_(l_*2.0*M_PI*startFrequency_) {}

  /*!
   * Constructor.
   * @param range          Range of the profile.
   * @param amplitude      Amplitude of the chirp.
   * @param startFrequency Starting frequency of the chirp.
   * @param endFrequency   Ending frequency of the chirp.
   */
  ProfileLogChirpUpSweep(const RangeT range, const PrimT amplitude,
                         const PrimT startFrequency, const PrimT endFrequency)
  : ProfileLogChirpUpSweep(range, 0.0, amplitude, startFrequency, endFrequency) {}

  virtual ~ProfileLogChirpUpSweep() {}

protected:
  PrimT getValueInRange(const PrimT time) const
  {
    const PrimT e = std::exp((time - this->range_.getStart())*lInv_);
    const PrimT m = k_*(e - 1);
    return offset_ + amplitude_*std::sin(m);
  }

  PrimT getFirstDerivativeInRange(const PrimT time) const
  {
    const PrimT e = std::exp((time - this->range_.getStart())*lInv_);
    const PrimT m = k_*(e - 1);
    return amplitude_*k_*lInv_*e*std::cos(m);
  }

  PrimT getSecondDerivativeInRange(const PrimT time) const
  {
    const PrimT e = std::exp((time - this->range_.getStart())*lInv_);
    const PrimT m = k_*(e - 1);
    return amplitude_*k_/std::pow(l_, 2.0)*e*(-k_*e*std::sin(m) + std::cos(m));
  }
};

template <typename PrimT_>
using ProfileLogChirpUpSweepPtr = std::shared_ptr<ProfileLogChirpUpSweep<PrimT_>>;

using ProfileLogChirpUpSweepF = ProfileLogChirpUpSweep<float>;
using ProfileLogChirpUpSweepD = ProfileLogChirpUpSweep<double>;
using ProfileLogChirpUpSweepFPtr = ProfileLogChirpUpSweepPtr<float>;
using ProfileLogChirpUpSweepDPtr = ProfileLogChirpUpSweepPtr<double>;


} // signal_generation
