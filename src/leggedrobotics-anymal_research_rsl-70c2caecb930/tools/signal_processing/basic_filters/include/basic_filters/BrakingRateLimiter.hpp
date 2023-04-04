/**
 * @authors     Aravind Vijayan
 * @affiliation ANYbotics
 * @brief       Rate limiting filter with control over brake towards zero.
 */

#pragma once

#include "basic_filters/helper_functions.hpp"
#include "basic_filters/traits/traits.hpp"
#include "basic_filters/typedefs.hpp"

namespace basic_filters {

/*!
 * \brief Computes the output such that the rate of change is limited. Further, it provides control over braking towards zero.
 *
 * This class implements a filter such that the input is rate limited,
 * i.e., the rate of change of the value will not exceed the defined maximum rate of change.
 * The filter allows to have a different rate of change when input value is in the direction of Zero, allowing for custom braking behavior.
 * The filter uses the time elapsed since the previous filter update(provided by the user) to compute the next filter update.
 * The input can be basic types or Eigen matrices.
 * \anchor logic_table
 * | Case   | Input To Filter |  Current Filter Value | Condition   | Behavior    |
 * | :----: | :-------------: | :-------------------: | :---------: | :---------: |
 * |   1    | \f$i >= 0\f$    | \f$v >= 0\f$          | \f$i = v\f$ | Remain same |
 * |   2    | \f$i >= 0\f$    | \f$v >= 0\f$          | \f$i > v\f$ | Accelerate  |
 * |   3    | \f$i >= 0\f$    | \f$v >= 0\f$          | \f$i < v\f$ | Brake       |
 * |   4    | \f$i < 0\f$     | \f$v < 0\f$           | \f$i = v\f$ | Remain same |
 * |   5    | \f$i < 0\f$     | \f$v < 0\f$           | \f$i < v\f$ | Accelerate  |
 * |   6    | \f$i < 0\f$     | \f$v < 0\f$           | \f$i > v\f$ | Brake       |
 * |   7    | \f$i >= 0\f$    | \f$v < 0\f$           |     -       | Brake       |
 * |   8    | \f$i < 0\f$     | \f$v >= 0\f$          |     -       | Brake       |
 * @tparam ValueType_ Can be an integer type or a statically sized \c Eigen matrix.
 */

template <typename ValueType_>
class BrakingRateLimiter {
 public:
  /*!
   * \brief Default Constructor.
   *
   * Constructs an uninitialized \c BrakeRateLimiter filter.
   * The \link BrakingRateLimiter::value_ value_\endlink , \link BrakingRateLimiter::maxRate_ maxRate_\endlink ,
   * and \link BrakingRateLimiter::maxBrakingRate_ maxBrakingRate_\endlink are assigned the default value of the @p ValueType_.
   */
  BrakingRateLimiter()
      : value_(getDefaultValue<ValueType_>()),
        initialized_(false),
        maxRate_(getDefaultValue<ValueType_>()),
        maxBrakingRate_(getDefaultValue<ValueType_>()) {}

  /*!
   * \brief Constructor to create the filter passing an initial value.
   *
   * Constructs an initialized \c BrakeRateLimiter filter.
   * The \link BrakingRateLimiter::maxRate_ maxRate_\endlink , and \link BrakingRateLimiter::maxBrakingRate_ maxBrakingRate_\endlink
   * are assigned the default value of the @p ValueType_.
   * @param initialValue Initial value of the rate limited value.
   */
  explicit BrakingRateLimiter(const ValueType_& initialValue)
      : value_(initialValue), initialized_(true), maxRate_(getDefaultValue<ValueType_>()), maxBrakingRate_(getDefaultValue<ValueType_>()) {}

  /*!
   * \brief Move constructor. \copybrief BrakingRateLimiter(const ValueType_&)
   *
   * \copydetails BrakingRateLimiter(const ValueType_&)
   */
  explicit BrakingRateLimiter(ValueType_&& initialValue)
      : value_(std::move(initialValue)),
        initialized_(true),
        maxRate_(getDefaultValue<ValueType_>()),
        maxBrakingRate_(getDefaultValue<ValueType_>()) {}

  /*!
   * \brief Constructor to create the filter passing an initial value and rate limits.
   *
   * @param initialValue Initial value of the rate limited value.
   * @param maxRate Maximum rate of change of the value.
   * @param maxBrakingRate Maximum braking rate of the value.
   * @note @p maxRate and @p maxBrakingRate must be positive.
   */
  BrakingRateLimiter(const ValueType_& initialValue, const ValueType_& maxRate, const ValueType_& maxBrakingRate)
      : value_(initialValue), initialized_(true), maxRate_(maxRate), maxBrakingRate_(maxBrakingRate) {}

  /*!
   * \brief Destructor.
   */
  ~BrakingRateLimiter() = default;

  /*!
   * \brief Set maximum rate and maximum breaking rate of the filter.
   *
   * @param maxRate Maximum rate of change of the value.
   * @param maxBrakingRate Maximum braking rate of the value.
   * @note @p maxRate and @p maxBrakingRate must be positive.
   */
  void setParameters(const ValueType_& maxRate, const ValueType_& maxBrakingRate) {
    maxRate_ = maxRate;
    maxBrakingRate_ = maxBrakingRate;
  }

  /*!
   * \brief  Initialize the filter.
   * @param initialValue initial value of the value to be filtered.
   */
  void initialize(const ValueType_& initialValue) {
    value_ = initialValue;
    initialized_ = true;
  }

  /*!
   * \brief Reset the filter.
   */
  void reset() {
    value_ = getDefaultValue<ValueType_>();
    initialized_ = false;
  }

  /*!
   * \brief Update step of the filter. Templated for updating \c Eigen types
   *
   * This function computes the rate limited output.
   * given the input value and time step.
   * @tparam V Type of the \c Eigen type filter input. Eg. \c Eigen::Vector3d.
   * @param input Input to the filter
   * @param dt time step.
   * @warning @p ValueType_ must be a statically sized \c Eigen matrix.
   */
  template <typename V = ValueType_>
  typename std::enable_if<traits::is_eigen_matrix<V>::value, void>::type update(const ValueType_& input, const double& dt) {
    static_assert(ValueType_::RowsAtCompileTime != Eigen::Dynamic, "BrakingRateLimiter expects Statically sized Eigen matrix");
    if (!initialized_) {
      initialize(input);
    } else {
      constexpr int numElements = ValueType_::RowsAtCompileTime * ValueType_::ColsAtCompileTime;
      for (auto i = 0; i < numElements; ++i) {
        value_.array()[i] =
            computeRateLimitedValue(input.array()[i], value_.array()[i], maxRate_.array()[i], maxBrakingRate_.array()[i], dt);
      }
    }
  }

  /*!
   * \brief Update step of the filter. Template for updating non eigen types
   *
   * This function computes the rate limited output.
   * given the input value and time step.
   * @tparam V Type of the Integer type filter input. eg. \c int , \c double , \c float etc.
   * @param input Input to the filter
   * @param dt time step.
   */
  template <typename V = ValueType_>
  typename std::enable_if<!traits::is_eigen_matrix<V>::value, void>::type update(const ValueType_& input, const double& dt) {
    if (!initialized_) {
      initialize(input);
    } else {
      value_ = computeRateLimitedValue(input, value_, maxRate_, maxBrakingRate_, dt);
    }
  }

  /*!
   * \brief Get value of the filter.
   *
   * @return Rate limited value.
   */
  inline const ValueType_& getValue() const { return value_; }

 private:
  /*!
   * \brief  Computes the rate limited value given the current value and the input.
   *
   * This function considers rate limiting and braking separately and computes the resultant filtered value.
   * For details on the rate selection algorithm, see \ref logic_table
   * @tparam ScalarValueType_ Scalar value type of the input.
   * @param input Input to the filter.
   * @param value Current value of the filter.
   * @param maxRate Maximum rate of change of the value.
   * @param maxBrakingRate Maximum braking rate of the value.
   * @param dt Time step.
   * @return Rate limited value.
   */
  template <typename ScalarValueType_>
  inline static ScalarValueType_ computeRateLimitedValue(const ScalarValueType_& input, const ScalarValueType_& value,
                                                         const ScalarValueType_& maxRate, const ScalarValueType_& maxBrakingRate,
                                                         const double& dt) {
    const auto change = input - value;
    auto rateLimitedChange = change;
    if (change * value < getDefaultValue<ScalarValueType_>()) {
      // Braking case
      if (std::abs(change) > dt * maxBrakingRate) {
        rateLimitedChange = dt * maxBrakingRate * change / std::abs(change);
      }

    } else {
      // Rate limited case
      if (std::abs(change) > dt * maxRate) {
        rateLimitedChange = dt * maxRate * change / std::abs(change);
      }
    }
    return value + rateLimitedChange;
  }

  //! Value to be filtered.
  ValueType_ value_;
  //! Indicator whether the filter has been initialized.
  bool initialized_;
  //! Maximum rate of change away from zero.
  ValueType_ maxRate_;
  //! Rate at which input changes towards zero.
  ValueType_ maxBrakingRate_;
};

}  // namespace basic_filters