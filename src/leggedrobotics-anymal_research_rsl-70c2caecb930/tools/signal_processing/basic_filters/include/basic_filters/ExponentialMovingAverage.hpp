/*!
 * @file 	ExponentialMovingAverage.hpp
 * @author 	Stephane Caron
 * @date        May, 2020
 * @note        Adapted from https://github.com/stephane-caron/lipm_walking_controller/ (BSD-2-Clause)
 * @version 1.0
 */

#pragma once

#include <message_logger/message_logger.hpp>

#include "basic_filters/helper_functions.hpp"
#include "basic_filters/traits/traits.hpp"
#include "basic_filters/typedefs.hpp"

namespace basic_filters {

/** Exponential Moving Average.
 * @brief Computes the exponential moving average of an input.
 *
 * This filter can be seen as an integrator:
 *
 * \f[
 *    y(t) = \frac{1}{T} \int_{\tau=0}^t x(\tau) e^{(\tau - t) / T} {\rm
 *    d}{\tau}
 * \f]
 *
 * with \f$T > 0\f$ a reset period acting as anti-windup. It can also
 * (informally) be interpreted as the average value of the input signal
 * \f$x(t)\f$ over the last \f$T\f$ seconds. Formally, it represents the
 * amount of time for the smoothed response of a unit input to reach \f$1-1/e
 * \ (\approx 63\%)\f$ of the original signal.
 *
 * See <https://en.wikipedia.org/wiki/Exponential_smoothing>. It is
 * equivalent to a [low-pass
 * filter](https://en.wikipedia.org/wiki/Low-pass_filter) applied to the
 * integral of the input signal.
 *
 */
template <typename ValueType_>
class ExponentialMovingAverage {
 public:
  /*!
   *  @brief Default constructor.
   *
   *  The default constructor sets @p dt_ and @p timeConstant_ to \f$1.0\f$.
   */
  ExponentialMovingAverage() : ExponentialMovingAverage(1., 1.) {}

  /*! @brief Constructor.
   *
   * @param dt Time in [s] between two readings.
   * @param timeConstant Informally, length of the recent-past window, in [s].
   * @param initValue Initial value of the output average.
   */
  explicit ExponentialMovingAverage(double dt, double timeConstant, const ValueType_& initValue = getDefaultValue<ValueType_>()) {
    setFilterParameters(dt, timeConstant, initValue);
  }

  //! Default destructor.
  ~ExponentialMovingAverage() = default;

  /*! @brief Reset filter.
   *
   * @param dt Time in [s] between two readings.
   * @param timeConstant Informally, length of the recent-past window, in [s].
   * @param resetValue Initial value of the output average.
   */
  void setFilterParameters(double dt, double timeConstant, const ValueType_& resetValue = getDefaultValue<ValueType_>()) {
    if (dt <= 0.) {
      MELO_WARN_STREAM("Invalid time step dt=" << dt << " [s]. Forcing to dt=1e-3 [s].")
      dt = 1e-3;
    }

    timeConstant = std::max(timeConstant, 2 * dt);  // Nyquist–Shannon sampling theorem
    alpha_ = 1. - std::exp(-dt / timeConstant);
    dt_ = dt;
    timeConstant_ = timeConstant;
    reset(resetValue);
  }

  /*!
   * @brief Reset the filter
   * @param resetValue    value to reset the filter to
   */
  void reset(const ValueType_& resetValue = getDefaultValue<ValueType_>()) { average_ = resetValue; }

  /*! @brief Process a new input.
   *
   * @param value new input Value
   * @return filtered value
   */
  ValueType_ advance(const ValueType_& value) {
    average_ += alpha_ * (value - average_);
    return getFilteredValue();
  }

  /*! @brief Get filter output.
   *
   * @return Last calculated output of the filter.
   */
  ValueType_ getFilteredValue() const { return average_; }

 private:
  //! Current filter output.
  ValueType_ average_ = {};

  //! Internal factor used for filter updates.
  double alpha_ = 1.;

  //! Time in [s] between two readings.
  double dt_ = 1.;

  //! Informally, length of the recent-past window, in [s].
  double timeConstant_ = 1.;
};

}  // namespace basic_filters
