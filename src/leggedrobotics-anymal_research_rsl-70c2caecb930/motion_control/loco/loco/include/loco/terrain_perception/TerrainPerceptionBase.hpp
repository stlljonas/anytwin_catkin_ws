/*
 * TerrainPerceptionBase.hpp
 *
 *  Created on: Apr 15, 2014
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/common/typedefs.hpp"

namespace loco {

/*! The role of terrain perception is to compute the control frame, as well as some torso/whole-body quantities in this control frame.
 *
 * Inputs:
 * - TBD by child class
 *
 * Outputs:
 * - torso->measuredState->inControlFrame->linearVelocity
 * - torso->measuredState->inControlFrame->angularVelocity
 * - wholeBody->wholeBodyMeasuredState->inControlFrame->positionControlToWholeBodyCom
 * - wholeBody->wholeBodyMeasuredState->inControlFrame->linearVelocityControlToWholeBodyCom
 *
 * Advanced by: main controller module.
 *
 */
class TerrainPerceptionBase : public ModuleBase {
 public:
  explicit TerrainPerceptionBase(const std::string& name = "");
  ~TerrainPerceptionBase() override = default;

  virtual void updateControlFrameOrigin() = 0;
  virtual void updateControlFrameAttitude() = 0;

  virtual void updateTorsoStateInControlFrame(TorsoBase& torso);
  virtual void updateWholeBodyStateInControlFrame(WholeBody& wholeBody);
};

} /* namespace loco */
