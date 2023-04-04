/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       Implementation of an average calculator.
 */

#pragma once

#include <ostream>
#include <string>

namespace average_calculator {

/*!
 * Class containing the algorithmic part of the package.
 */
class AverageCalculator {
 public:
  /*!
   * Constructor.
   */
  AverageCalculator() = default;

  /*!
   * Destructor.
   */
  ~AverageCalculator() = default;

  /*!
   * Add a new value.
   * @param value New value to add.
   */
  void addValue(double value);

  /*!
   * Clear all data.
   */
  void clear();

  /*!
   * Get the computed average of the values.
   * Note: The definition of trivial setter/getter methods should be put in the header file.
   * @return The average of the values.
   */
  double getAverageValue() const { return averageValue_; }

  /*!
   * Print internal state to ostream.
   */
  std::string toString() const;

 private:
  //! Internal variable to hold the current average.
  double averageValue_ = 0.0;

  //! Number of values taken.
  unsigned int nValues_ = 0;
};

/*!
 * Overload ostream operator<< for convenient use with the AverageCalculator class.
 * Note 1: The intermediate use of toString() allows us to avoid declaring this function a friend of AverageCalculator
 * for acces to its members. In addition, when deriving from AverageCalculator, this would leave us with all advantages of
 * polymorphism, since, for a different output, one would only need to override the toString() member function.
 * Note 2: The inline specifier is required to let the linker (which would otherwise throw an error) know that it is okay
 * that this header-only function may appear in multiple translation units. However, it does not determine whether the
 * compiler will inline the function.
 */
inline std::ostream& operator<<(std::ostream& os, const AverageCalculator& averageCalculator) {
  return os << averageCalculator.toString();
}

}  // namespace average_calculator
