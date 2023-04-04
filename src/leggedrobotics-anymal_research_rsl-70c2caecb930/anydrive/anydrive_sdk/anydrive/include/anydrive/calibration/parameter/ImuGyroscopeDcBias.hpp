#pragma once

#include <cstdint>

namespace anydrive {
namespace calibration {
namespace parameter {

struct ImuGyroscopeDcBias {
  int32_t x_ = 0;
  int32_t y_ = 0;
  int32_t z_ = 0;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const ImuGyroscopeDcBias& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
