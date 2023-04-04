/*
 * TerrainModelFreePlane.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/common/TerrainModelFreePlane.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

namespace loco {

TerrainModelFreePlane::TerrainModelFreePlane()
    : TerrainModelPlane(),
      frictionCoefficientBetweenTerrainAndFoot_(0.6),
      positionInWorldFrame_(loco::Vector::Zero()),
      normalInWorldFrame_(loco::Vector::UnitZ()),
      orientationPlaneToWorld_() {}  // constructor

bool TerrainModelFreePlane::initialize(double dt) {
  // Initialize the plane as coincident with the ground
  normalInWorldFrame_ = loco::Vector::UnitZ();
  positionInWorldFrame_.setZero();

  return true;
}  // initialize

bool TerrainModelFreePlane::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle terrainModelHandle = handle;
  if (!tinyxml_tools::getChildHandle(terrainModelHandle, handle, "TerrainModel")) {
    return false;
  }

  TiXmlHandle parametersHandle = handle;
  if (!tinyxml_tools::getChildHandle(parametersHandle, terrainModelHandle, "Parameters")) {
    return false;
  }

  if (!tinyxml_tools::loadParameter(frictionCoefficientBetweenTerrainAndFoot_, parametersHandle, "frictionCoefficient")) {
    return false;
  }

  return true;
}

bool TerrainModelFreePlane::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(normalInWorldFrame_, "normalInWorldFrame", ns);
  signal_logger::add(positionInWorldFrame_, "positionInWorldFrame", ns);
  signal_logger::add(orientationPlaneToWorld_, "orientationPlaneToWorld", ns);
  return true;
}

void TerrainModelFreePlane::setNormalandPositionInWorldFrame(const loco::Vector& normal, const loco::Position& position) {
  normalInWorldFrame_ = normal;
  positionInWorldFrame_ = position;
  Eigen::Vector3d axisZ = Eigen::Vector3d::UnitZ();
  try {
    orientationPlaneToWorld_.setFromVectors(normalInWorldFrame_.toImplementation(), axisZ).setUnique();
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in TerrainModelFreePlane::setNormalandPositionInWorldFrame()." << std::endl);
    orientationPlaneToWorld_.setIdentity();
  }

}  // set normal and position

bool TerrainModelFreePlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const {
  // For a plane, the normal is constant (independent from the position at which it is evaluated)
  normalInWorldFrame = normalInWorldFrame_;
  normalInWorldFrame.normalize();
  return true;
}  // get normal

bool TerrainModelFreePlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {
  /* Dividing by normalInWorldFrame.z() is safe because the plane equation is z = d-ax-by.
   * If the normal is normalized, its z component will still be greater than zero
   */
  positionWorldToLocationInWorldFrame.z() =
      positionInWorldFrame_.z() + normalInWorldFrame_.x() * (positionInWorldFrame_.x() - positionWorldToLocationInWorldFrame.x()) +
      normalInWorldFrame_.y() * (positionInWorldFrame_.y() - positionWorldToLocationInWorldFrame.y());
  positionWorldToLocationInWorldFrame.z() /= normalInWorldFrame_.z();

  return true;
}  // get height at position, update position

bool TerrainModelFreePlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  /* Dividing by normalInWorldFrame.z() is safe because the plane equation is z = d-ax-by.
   * If the normal is normalized, its z component will still be greater than zero
   */
  heightInWorldFrame = positionInWorldFrame_.z() +
                       normalInWorldFrame_.x() * (positionInWorldFrame_.x() - positionWorldToLocationInWorldFrame.x()) +
                       normalInWorldFrame_.y() * (positionInWorldFrame_.y() - positionWorldToLocationInWorldFrame.y());
  heightInWorldFrame /= normalInWorldFrame_.z();

  return true;
}  // get height at position, return height

bool TerrainModelFreePlane::getMaxHeightBetweenTwoPoints(const loco::Position& point1, const loco::Position& point2, double& height) const {
  if (point1.z() > point2.z()) {
    height = point1.z();
  } else {
    height = point2.z();
  }
  return true;
}

bool TerrainModelFreePlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame,
                                                          double& frictionCoefficient) const {
  frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
  return true;
}  // get friction

Position TerrainModelFreePlane::getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(
    const Position& positionWorldToLocationInWorldFrame) const {
  // For theory background see Papula, "Mathematische Formelasammlung", pag. 62
  Vector n;
  this->getNormal(positionWorldToLocationInWorldFrame, n);

  return positionWorldToLocationInWorldFrame - getHeightAboveTerrainAlongSurfaceNormal(positionWorldToLocationInWorldFrame) * Position(n);
}

double TerrainModelFreePlane::getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(
    const Position& positionWorldToLocationInWorldFrame) const {
  return fabs(getHeightAboveTerrainAlongSurfaceNormal(positionWorldToLocationInWorldFrame));
}

double TerrainModelFreePlane::getHeightAboveTerrainAlongSurfaceNormal(const Position& positionWorldToLocationInWorldFrame) const {
  // For theory background see Papula, "Mathematische Formelasammlung", pag. 62
  Vector n;
  this->getNormal(positionWorldToLocationInWorldFrame, n);

  Position r1, rq;
  r1 = Position::Zero();
  this->getHeight(r1);
  rq = positionWorldToLocationInWorldFrame;
  Vector r1q(rq - r1);

  return n.dot(r1q) / n.norm();
}

} /* namespace loco */
