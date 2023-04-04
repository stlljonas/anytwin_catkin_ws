/**
 * @authors     jelavice, Aravind Vijayan
 * @affiliation ANYbotics
 * @brief       Filter to limit the rate of change of a value.
 */

#pragma once

#include "basic_filters/helper_functions.hpp"
#include "basic_filters/traits/traits.hpp"
#include "basic_filters/typedefs.hpp"

/*!
 * Rate limiter Filter
 * \brief Computes the output such that the rate of change is limited.
 *
 * This class implements a filter such that the input is rate limited,
 * i.e., the rate of change of the value will not exceed the defined maximum rate of increase and maximum rate of decrease.
 * The filter uses the time elapsed since the previous filter update(provided by the user) to compute the next filter update.
 * The input can be basic types or Eigen matrices.
 */
namespace basic_filters {

template <typename ValueType_>
class RateLimiter {
 public:
  /*!
   * Default Constructor.
   */
  RateLimiter()
      : value_(getDefaultValue<ValueType_>()),
        initialized_(false),
        maxRisingSlope_(getDefaultValue<ValueType_>()),
        maxFallingSlope_(getDefaultValue<ValueType_>()) {}

  /*!
   * Constructor.
   * @param initialValue Initial value of the rate limited value.
   */
  explicit RateLimiter(const ValueType_& initialValue)
      : value_(initialValue),
        initialized_(true),
        maxRisingSlope_(getDefaultValue<ValueType_>()),
        maxFallingSlope_(getDefaultValue<ValueType_>()) {}

  /*!
   * Constructor.
   * @param initialValue Initial value of the rate limited value.
   */
  explicit RateLimiter(ValueType_&& initialValue)
      : value_(std::move(initialValue)),
        initialized_(true),
        maxRisingSlope_(getDefaultValue<ValueType_>()),
        maxFallingSlope_(getDefaultValue<ValueType_>()) {}

  /*!
   * Constructor.
   * @param initialValue Initial value of the rate limited value.
   * @param maxFallingSlope Maximum rate of increase of the value.
   * @param maxRisingSlope Maximum rate of decrease of the value.
   */
  RateLimiter(const ValueType_& initialValue, const ValueType_& maxRisingSlope, const ValueType_& maxFallingSlope)
      : value_(initialValue), initialized_(true), maxRisingSlope_(maxRisingSlope), maxFallingSlope_(maxFallingSlope) {}

  /*!
   * Destructor.
   */
  ~RateLimiter() = default;

  /*!
   * Set paramters of the filter.
   * @param maxFallingSlope Maximum rate of increase of the value.
   * @param maxRisingSlope Maximum rate of decrease of the value.
   */
  void setParameters(const ValueType_& maxRisingSlope, const ValueType_& maxFallingSlope) {
    maxFallingSlope_ = maxFallingSlope;
    maxRisingSlope_ = maxRisingSlope;
  }

  /*!
   * Initialize the filter.
   * @param initialValue initial value of the value to be filtered.
   */
  void initialize(const ValueType_& initialValue) {
    value_ = initialValue;
    initialized_ = true;
  }

  /*!
   * Reset the filter.
   */
  void reset() {
    value_ = getDefaultValue<ValueType_>();
    initialized_ = false;
  }

  /*!
   * Update step of the filter. This function computes the rate limited output
   * given the input value and time step.
   * @tparam V Type of the filter input. Eg. double, Eigen::VectorXd etc.
   * @param input Input to the filter
   * @param dt time step.
   */
  // Template for updating eigen types
  template <typename V = ValueType_>
  typename std::enable_if<traits::is_eigen_matrix<V>::value, void>::type update(const ValueType_& input, const double& dt) {
    if (!initialized_) {
      initialize(input);
    } else {
      constexpr int numElements = ValueType_::RowsAtCompileTime * ValueType_::ColsAtCompileTime;
      for (auto i = 0; i < numElements; ++i) {
        const typename V::Scalar upperLimit = value_.array()[i] + dt * maxRisingSlope_.array()[i];
        const typename V::Scalar lowerLimit = value_.array()[i] + dt * maxFallingSlope_.array()[i];
        value_.array()[i] = std::max(lowerLimit, std::min(upperLimit, input.array()[i]));
      }
    }
  }

  // Template for updating non eigen types
  template <typename V = ValueType_>
  typename std::enable_if<!traits::is_eigen_matrix<V>::value, void>::type update(const ValueType_& input, const double& dt) {
    if (!initialized_) {
      initialize(input);
    } else {
      const ValueType_ upperLimit = value_ + dt * maxRisingSlope_;
      const ValueType_ lowerLimit = value_ + dt * maxFallingSlope_;
      value_ = std::max(lowerLimit, std::min(upperLimit, input));
    }
  }

  /*!
   * Get value from the filter.
   * @return Rate limited value.
   */
  inline const ValueType_& getValue() const { return value_; }

 private:
  //! Value to be filtered.
  ValueType_ value_;
  //! Indicator whether the filter has been initialized.
  bool initialized_;
  //! Maximum rate of increase.
  ValueType_ maxRisingSlope_;
  //! Maximum rate of decrease.
  ValueType_ maxFallingSlope_;
};

}  // namespace basic_filters