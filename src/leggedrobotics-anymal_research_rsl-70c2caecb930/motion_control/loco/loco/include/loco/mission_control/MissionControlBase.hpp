/*
 * MissionControlBase.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/typedefs.hpp"

// tiny xml
#include <tinyxml.h>

namespace loco {

class MissionControlBase : public ModuleBase {
 public:
  MissionControlBase();
  ~MissionControlBase() override = default;

  virtual const Twist& getDesiredBaseTwistInControlFrame() const = 0;
  virtual const Twist& getMaximumBaseTwistInControlFrame() const = 0;
  virtual const Pose& getMinimalPoseOffset() const = 0;
  virtual const Pose& getMaximalPoseOffset() const = 0;

  /*! Computes an interpolated version of the two mission controllers passed in as parameters.
   *  If t is 0, the current setting is set to missionController1, 1 -> missionController2, and values in between
   *  correspond to interpolated parameter set.
   * @param missionController1
   * @param missionController2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2, double t);
};

} /* namespace loco */
