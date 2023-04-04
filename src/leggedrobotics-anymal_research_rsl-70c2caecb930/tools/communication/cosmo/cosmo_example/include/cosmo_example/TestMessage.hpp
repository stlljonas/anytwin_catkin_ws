//
// Created by pleemann on 25.05.17.
//

#pragma once

#include <array>

class TestMessage {
 public:
  TestMessage() : a_(0), b_() {}
  int a_;

  // std::array is the only container which is compatible with cosmo, do not use the other containers or std::strings!
  std::array<double, 5> b_;
};
