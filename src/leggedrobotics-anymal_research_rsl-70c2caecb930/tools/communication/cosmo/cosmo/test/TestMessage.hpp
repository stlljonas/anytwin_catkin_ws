//
// Created by pleemann on 25.05.17.
//

#pragma once

#include <cstdint>

struct TestMessage {
  int64_t timestamp_;
  double position_[3];
  double orientation_[4];
  int32_t numRanges_;
  bool enabled_;
};
