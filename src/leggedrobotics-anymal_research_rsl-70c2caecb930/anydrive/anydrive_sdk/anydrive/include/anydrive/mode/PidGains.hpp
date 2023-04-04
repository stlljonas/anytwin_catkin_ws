#pragma once

#include <cmath>

namespace anydrive {
namespace mode {

template <typename T>
class PidGains {
 protected:
  T p_ = 0.0;
  T i_ = 0.0;
  T d_ = 0.0;

 public:
  PidGains() = default;

  PidGains(T p, T i, T d) : p_(p), i_(i), d_(d) {}

  virtual ~PidGains() = default;

  T getP() const { return p_; }

  T& getP() { return p_; }

  void setP(T p) { p_ = p; }

  T getI() const { return i_; }

  T& getI() { return i_; }

  void setI(T i) { i_ = i; }

  T getD() const { return d_; }

  T& getD() { return d_; }

  void setD(T d) { d_ = d; }

  PidGains operator*(const double scaling) const { return PidGains(p_ * scaling, i_ * scaling, d_ * scaling); }

  PidGains& operator*=(const double scaling) {
    *this = *this * scaling;
    return *this;
  }

  bool isValid() const { return (std::isfinite(p_) && std::isfinite(i_) && std::isfinite(d_)); }
};

template <typename T>
std::ostream& operator<<(std::ostream& out, const PidGains<T>& command) {
  return out << command.getP() << ", " << command.getI() << ", " << command.getD();
}

using PidGainsF = PidGains<float>;
using PidGainsD = PidGains<double>;

}  // namespace mode
}  // namespace anydrive
