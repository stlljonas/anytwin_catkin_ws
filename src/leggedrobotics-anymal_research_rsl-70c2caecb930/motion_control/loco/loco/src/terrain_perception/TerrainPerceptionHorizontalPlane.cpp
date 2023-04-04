/*
 * TerrainPerceptionHorizontalPlane.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: Christian Gehring
 */

#include "loco/terrain_perception/TerrainPerceptionHorizontalPlane.hpp"

namespace loco {

TerrainPerceptionHorizontalPlane::TerrainPerceptionHorizontalPlane(TerrainModelHorizontalPlane& terrainModel, WholeBody& wholeBody)
    : TerrainPerceptionBase(), terrainModel_(terrainModel), torso_(*wholeBody.getTorsoPtr()), legs_(*wholeBody.getLegsPtr()) {}

bool TerrainPerceptionHorizontalPlane::initialize(double dt) {
  return advance(dt);
}

bool TerrainPerceptionHorizontalPlane::advance(double dt) {
  int groundedLimbCount = 0;
  double gHeight = 0.0;

  for (auto leg : legs_) {
    if (leg->getContactSchedule().isAndShouldBeGrounded()) {
      groundedLimbCount++;
      gHeight += leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame().z();
    }
  }

  if (groundedLimbCount > 0) {
    terrainModel_.setHeight(gHeight / groundedLimbCount);
  }

  updateControlFrameOrigin();
  updateControlFrameAttitude();
  updateTorsoStateInControlFrame(torso_);
  return true;
}

void TerrainPerceptionHorizontalPlane::updateControlFrameOrigin() {
  //--- Position of the control frame is equal to the position of the world frame.
  torso_.getMeasuredStatePtr()->inControlFrame().setPositionWorldToControlInWorldFrame(Position::Zero());
  //---
}

void TerrainPerceptionHorizontalPlane::updateControlFrameAttitude() {
  //--- hack set control frame equal to heading frame
  RotationQuaternion orientationWorldToControl;
  EulerAnglesZyx orientationWorldToHeadingEulerZyx = EulerAnglesZyx(torso_.getMeasuredState().getOrientationWorldToBase()).getUnique();
  orientationWorldToHeadingEulerZyx.setPitch(0.0);
  orientationWorldToHeadingEulerZyx.setRoll(0.0);
  orientationWorldToControl = RotationQuaternion(orientationWorldToHeadingEulerZyx.getUnique());

  torso_.getMeasuredStatePtr()->inControlFrame().setOrientationWorldToControl(orientationWorldToControl);
  torso_.getMeasuredStatePtr()->inControlFrame().setPositionControlToBaseInControlFrame(
      orientationWorldToControl.rotate(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame() -
                                       torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame()));

  const RotationQuaternion& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  torso_.getMeasuredStatePtr()->inControlFrame().setOrientationControlToBase(orientationWorldToBase * orientationWorldToControl.inverted());
}

} /* namespace loco */
