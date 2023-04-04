#pragma once

#include <cstdint>

namespace anydrive {
namespace calibration {
namespace parameter {

struct SpringStiffness {
  //! Spring stiffness for negative deflections [Nm/rad].
  float neg_ = 0.0;
  //! Spring stiffness for positive deflections [Nm/rad].
  float pos_ = 0.0;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const SpringStiffness& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
