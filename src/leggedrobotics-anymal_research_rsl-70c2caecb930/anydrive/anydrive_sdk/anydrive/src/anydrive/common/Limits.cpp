#include "anydrive/common/Limits.hpp"
#include "anydrive/common/Macros.hpp"

namespace anydrive {
namespace common {

Limits::Limits(const double min, const double max) : min_(min), max_(max) {
  if (max < min) {
    ANYDRIVE_WARN("The interval is an empty set, since max (" << max << ") is smaller than min (" << min << ").");
  }
}

bool Limits::areInf() const {
  return (min_ == 0.0) && (max_ == 0.0);
}

double Limits::min() const {
  return min_;
}

double& Limits::min() {
  return min_;
}

double Limits::max() const {
  return max_;
}

double& Limits::max() {
  return max_;
}

bool Limits::liesWithin(const double value) const {
  return areInf() || ((min_ <= value) && (value <= max_));
}

Limits Limits::operator+(const double offset) const {
  return areInf() ? *this : Limits(min_ + offset, max_ + offset);
}

Limits& Limits::operator+=(const double offset) {
  *this = *this + offset;
  return *this;
}

Limits Limits::operator-(const double offset) const {
  return *this + -offset;
}

Limits& Limits::operator-=(const double offset) {
  *this = *this - offset;
  return *this;
}

Limits Limits::operator*(const double scaling) const {
  if (areInf()) {
    return *this;
  } else if (scaling >= 0) {
    return Limits(min_ * scaling, max_ * scaling);
  } else {
    return Limits(max_ * scaling, min_ * scaling);
  }
}

Limits& Limits::operator*=(const double scaling) {
  *this = *this * scaling;
  return *this;
}

}  // namespace common
}  // namespace anydrive
