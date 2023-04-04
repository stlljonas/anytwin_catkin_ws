/*
 * CumulativeMovingAverageFilter.hpp
 *
 *  Created on: Feb, 2017
 *      Author: Christian Gehring
 */

#pragma once

namespace basic_filters {
//! Cumulative Moving Average Filter
/*! Assumes that value has method setZero().
 * Used with kindr objects.
 */
template <typename Value_>
class CumulativeMovingAverageFilter {
 public:
  using Scalar = typename Value_::Scalar;

 public:
  CumulativeMovingAverageFilter() = default;
  virtual ~CumulativeMovingAverageFilter() = default;

  void reset() {
    numCmaPointsReceived_ = 0u;
    cumulativeMovingAverage_.setZero();
  }

  void update(const Value_& datum) {
    cumulativeMovingAverage_ = (datum + cumulativeMovingAverage_ * static_cast<Scalar>(numCmaPointsReceived_)) /
                               (static_cast<Scalar>(numCmaPointsReceived_) + static_cast<Scalar>(1.0));
    numCmaPointsReceived_++;
  }

  const Value_& getValue() const { return cumulativeMovingAverage_; }

  unsigned int getNumSamples() const { return numCmaPointsReceived_; }

 protected:
  Value_ cumulativeMovingAverage_;
  unsigned int numCmaPointsReceived_ = 0u;
};

}  // namespace basic_filters
