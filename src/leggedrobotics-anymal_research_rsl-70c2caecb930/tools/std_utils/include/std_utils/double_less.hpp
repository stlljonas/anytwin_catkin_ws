/*!
 * @file	 double_less.hpp
 * @author Gabriel Hottiger
 * @date	 Dec 4, 2017
 */
#pragma once

#include <cmath>
#include <functional>

namespace std_utils {
class double_less : public std::binary_function<double, double, bool> {  // NOLINT(readability-identifier-naming)
 public:
  explicit double_less(double arg_ = 1e-10) : epsilon_(arg_) {}
  bool operator()(const double& left, const double& right) const {
    // you can choose other way to make decision
    // (The original version is: return left < right;)
    return (std::fabs(left - right) > epsilon_) && (left < right);
  }

 private:
  double epsilon_;
};

}  // namespace std_utils
