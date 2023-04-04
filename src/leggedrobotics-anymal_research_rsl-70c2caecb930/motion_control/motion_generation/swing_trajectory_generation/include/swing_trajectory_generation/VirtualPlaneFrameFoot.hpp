/*
 * VirtualPlaneFrameFoot.hpp
 *
 *  Created on: Jan. 19, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion generation utils
#include "motion_generation_utils/typedefs.hpp"
#include "motion_generation_utils/VirtualPlaneFrameBase.hpp"


namespace sto {

class VirtualPlaneFrameFoot: public motion_generation::VirtualPlaneFrameBase {
 public:
  VirtualPlaneFrameFoot() = default;
  ~VirtualPlaneFrameFoot() override = default;

  //! Update components of the virtual plane frame.
  bool computeVirtualPlaneFrame(
      const motion_generation::Position& positionWorldToPreviousStanceFootholdInWorldFrame,
      const motion_generation::RotationQuaternion& orientationWorldToControl);
};

} /* namespace sto */
