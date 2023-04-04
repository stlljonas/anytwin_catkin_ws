/*
 * FirstOrderDerivativeTest.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: Philipp Leemann
 */

// gtest
#include <gtest/gtest.h>

// basic filters
#include "basic_filters/FirstOrderDerivative.hpp"

TEST(FirstOrderDerivative, DoubleFilterOutputTest) {  // NOLINT
#include "test_data_first_order_derivative.h"

  basic_filters::FirstOrderDerivative<double> filter(dt, tau, gain);

  for (unsigned int i = 0; i < len; ++i) {
    double y_out = filter.advance(u[i]);
    //        std::cout << std::fixed << std::setprecision(15) << y_out << ", ";
    ASSERT_NEAR(y[i], y_out, 5e-5);  // have to use low precision because matlab generated data files contain doubles converted to strings
  }
}
