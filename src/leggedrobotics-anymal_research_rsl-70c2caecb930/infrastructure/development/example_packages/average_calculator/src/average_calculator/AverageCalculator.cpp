/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       Implementation of an average calculator.
 */

#include "average_calculator/AverageCalculator.hpp"

namespace average_calculator {

void AverageCalculator::addValue(const double value) {
  averageValue_ = (nValues_ * averageValue_ + value) / (nValues_ + 1);
  ++nValues_;
}

void AverageCalculator::clear() {
  averageValue_ = 0.0;
  nValues_ = 0;
}

std::string AverageCalculator::toString() const {
  return "Average value: " + std::to_string(averageValue_) + "\nNumber of added values: " + std::to_string(nValues_);
}

}  // namespace average_calculator
