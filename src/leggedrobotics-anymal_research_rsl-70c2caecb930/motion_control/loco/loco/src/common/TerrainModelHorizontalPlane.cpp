/*
 * TerrainModelHorizontalPlane.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: Christian Gehring
 */

// loco
#include "loco/common/TerrainModelHorizontalPlane.hpp"

namespace loco {

TerrainModelHorizontalPlane::TerrainModelHorizontalPlane()
    : TerrainModelBase(),
      heightInWorldFrame_(0.0),
      normalInWorldFrame_(loco::Vector::UnitZ()),
      frictionCoefficientBetweenTerrainAndFoot_(0.8) {}

bool TerrainModelHorizontalPlane::initialize(double dt) {
  heightInWorldFrame_ = 0.0;
  normalInWorldFrame_ = loco::Vector::UnitZ();
  return true;
}

bool TerrainModelHorizontalPlane::loadParameters(const TiXmlHandle& handle) {
  /* gait pattern */
  TiXmlHandle handleToTerrainParameters(handle.FirstChild("TerrainModel").FirstChild("Parameters"));

  TiXmlElement* pElem = handleToTerrainParameters.Element();
  if (pElem == nullptr) {
    printf("[TerrainModelFreePlane::loadParameters] Could not find TerrainModel section in parameter file.\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("frictionCoefficient", &frictionCoefficientBetweenTerrainAndFoot_) != TIXML_SUCCESS) {
    printf("[TerrainModelHorizontalPlane::loadParameters] Could not find TerrainModel::frictionCoefficient in parameter file.\n");
    return false;
  }

  return true;
}

bool TerrainModelHorizontalPlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame,
                                            loco::Vector& normalInWorldFrame) const {
  normalInWorldFrame = normalInWorldFrame_;
  return true;
}

bool TerrainModelHorizontalPlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {
  positionWorldToLocationInWorldFrame.z() = heightInWorldFrame_;
  return true;
}

bool TerrainModelHorizontalPlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  heightInWorldFrame = heightInWorldFrame_;
  return true;
}

void TerrainModelHorizontalPlane::setHeight(double heightInWorldFrame) {
  heightInWorldFrame_ = heightInWorldFrame;
}

bool TerrainModelHorizontalPlane::getMaxHeightBetweenTwoPoints(const loco::Position& point1, const loco::Position& point2,
                                                               double& height) const {
  return true;
}

bool TerrainModelHorizontalPlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame,
                                                                double& frictionCoefficient) const {
  frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
  return true;
}

Position TerrainModelHorizontalPlane::getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
    const Position& positionWorldToLocationInWorldFrame) const {
  Position position = positionWorldToLocationInWorldFrame;
  getHeight(position);
  return position;
}

double TerrainModelHorizontalPlane::getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(
    const Position& positionWorldToLocationInWorldFrame) const {
  return fabs(getHeightAboveTerrainAlongSurfaceNormal(positionWorldToLocationInWorldFrame));
}

double TerrainModelHorizontalPlane::getHeightAboveTerrainAlongSurfaceNormal(const Position& positionWorldToLocationInWorldFrame) const {
  double height;
  getHeight(positionWorldToLocationInWorldFrame, height);
  return (positionWorldToLocationInWorldFrame.z() - height);
}

} /* namespace loco */
