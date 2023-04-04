/*
 * TerrainModelBase.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: Christian Gehring
 */

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "message_logger/message_logger.hpp"

namespace loco {

// Todo: move this to TerrainModelPlane
bool TerrainModelBase::getTerrainOrientation(const Position& positionWorldToLocationInWorldFrame,
                                             const Vector& vectorHeadingDirectionInWorldFrame, double& terrainPitch,
                                             double& terrainRoll) const {
  // position at which we calculate the terrain orientation.
  const Position positionWorldToBaseOnGroundInWorldFrame =
      getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(positionWorldToLocationInWorldFrame);

  // Compute orientation of w.r.t. to world x-axis
  RotationQuaternion orientationWorldToHeading;
  try {
    orientationWorldToHeading.setFromVectors(vectorHeadingDirectionInWorldFrame.toImplementation(),
                                             loco::Vector::UnitX().toImplementation());
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in TerrainModelBase::getTerrainOrientation()." << std::endl);
    orientationWorldToHeading.setIdentity();
  }

  // Get plane normal and align it with heading direction.
  Vector planeNormalInWorldFrame;
  bool success = getNormal(positionWorldToBaseOnGroundInWorldFrame, planeNormalInWorldFrame);
  const Vector normalInHeadingFrame = orientationWorldToHeading.rotate(planeNormalInWorldFrame);

  // get pitch and roll
  terrainPitch = std::atan2(normalInHeadingFrame.x(), normalInHeadingFrame.z());
  terrainRoll = std::atan2(normalInHeadingFrame.y(), normalInHeadingFrame.z());

  return success;
}

} /* namespace loco */
