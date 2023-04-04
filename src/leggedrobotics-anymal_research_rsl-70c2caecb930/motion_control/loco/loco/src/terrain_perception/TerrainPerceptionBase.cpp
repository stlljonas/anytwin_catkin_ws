/*
 * TerrainPerceptionBase.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: Christian Gehring
 */

// loco
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"

namespace loco {

TerrainPerceptionBase::TerrainPerceptionBase(const std::string& name) : ModuleBase(name) {}

void TerrainPerceptionBase::updateTorsoStateInControlFrame(TorsoBase& torso) {
  auto measuredState = torso.getMeasuredStatePtr();
  auto& measuredStateInControlFrame = measuredState->inControlFrame();
  auto& orientationControlToBase = measuredStateInControlFrame.getOrientationControlToBase();
  measuredStateInControlFrame.setLinearVelocityBaseInControlFrame(
      orientationControlToBase.inverseRotate(measuredState->getLinearVelocityBaseInBaseFrame()));
  measuredStateInControlFrame.setAngularVelocityBaseInControlFrame(
      orientationControlToBase.inverseRotate(measuredState->getAngularVelocityBaseInBaseFrame()));
}

void TerrainPerceptionBase::updateWholeBodyStateInControlFrame(WholeBody& wholeBody) {
  auto measuredStatePtr = wholeBody.getWholeBodyStateMeasuredPtr();
  const auto& measuredStateInControlFrame = wholeBody.getTorsoPtr()->getMeasuredState().inControlFrame();
  const auto& orientationWorldToControl = measuredStateInControlFrame.getOrientationWorldToControl();
  const Position positionControlToComInWorldFrame = measuredStatePtr->getPositionWorldToWholeBodyCenterOfMassInWorldFrame() -
                                                    measuredStateInControlFrame.getPositionWorldToControlInWorldFrame();

  measuredStatePtr->setPositionControlToWholeBodyCenterOfMassInControlFrame(
      orientationWorldToControl.rotate(positionControlToComInWorldFrame));
  measuredStatePtr->setLinearVelocityWholeBodyCenterOfMassInControlFrame(
      orientationWorldToControl.rotate(measuredStatePtr->getLinearVelocityWholeBodyCenterOfMassInWorldFrame()));
}

} /* namespace loco */
