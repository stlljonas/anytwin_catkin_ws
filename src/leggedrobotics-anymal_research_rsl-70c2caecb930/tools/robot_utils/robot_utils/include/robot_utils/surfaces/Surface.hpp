/*
 * Surface.hpp
 *
 *  Created on: Mar 21, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// kindr
#include "kindr/Core"

namespace robot_utils {

class Surface {
 public:
  using Vector = kindr::VectorTypeless3D;

  Surface() = default;
  virtual ~Surface() = default;
};

}  // namespace robot_utils
