#pragma once

#include <limits>

namespace anydrive {
namespace common {

//! Class implementing limits.
class Limits {
 protected:
  // The limits are an interval [min, max].
  // If min == max == 0.0, it is infinity, [-inf, inf].

  //! Lower limit.
  double min_ = 0.0;
  //! Upper limit.
  double max_ = 0.0;

 public:
  /*!
   * Constructor, setting limits to infinity.
   */
  Limits() = default;

  /*!
   * Constructor setting the limits
   * @param min Lower limit.
   * @param max Upper limit.
   */
  Limits(const double min, const double max);

  /*!
   * Destructor.
   */
  virtual ~Limits() = default;

  /*!
   * Check if the limits are infinity.
   * @return True iff the limits are infinity.
   */
  bool areInf() const;

  /*!
   * Get the lower limit.
   * @return Lower limit.
   */
  double min() const;

  /*!
   * Get the lower limit by reference.
   * @return Reference to the lower limit.
   */
  double& min();

  /*!
   * Get the upper limit.
   * @return Upper limit.
   */
  double max() const;

  /*!
   * Get the upper limit by reference.
   * @return Reference to the upper limit.
   */
  double& max();

  /*!
   * Check if a value lies within the limits.
   * @param value Value.
   * @return True iff the value lies within the limits.
   */
  bool liesWithin(const double value) const;

  /*!
   * Create new shifted the limits.
   * @param offset Offset to shift.
   * @return Shifted limits.
   */
  Limits operator+(const double offset) const;

  /*!
   * Shift limits by an offset.
   * @param offset Offset.
   * @return Shifted limits.
   */
  Limits& operator+=(const double offset);

  /*!
   * Create new shifted the limits.
   * @param offset Offset to shift.
   * @return Shifted limits.
   */
  Limits operator-(const double offset) const;

  /*!
   * Shift limits by an offset.
   * @param offset Offset.
   * @return Reference to shifted limits.
   */
  Limits& operator-=(const double offset);

  /*!
   * Create new scaled limits.
   * @param scaling Scaling factor.
   * @return Scaled limits.
   */
  Limits operator*(const double scaling) const;

  /*!
   * Scale limits by a factor.
   * @param scaling Scaling factor.
   * @return Reference to scaled limits.
   */
  Limits& operator*=(const double scaling);
};

}  // namespace common
}  // namespace anydrive
