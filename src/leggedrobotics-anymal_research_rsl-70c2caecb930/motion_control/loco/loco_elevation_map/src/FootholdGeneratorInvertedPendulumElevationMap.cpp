/*
 * FootholdGeneratorInvertedPendulum.cpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso, Christian Gehring
 */

// loco
#include "loco_elevation_map/FootholdGeneratorInvertedPendulumElevationMap.hpp"

// message logger
#include <message_logger/message_logger.hpp>


using namespace message_logger::log;


namespace loco {

FootholdGeneratorInvertedPendulumElevationMap::FootholdGeneratorInvertedPendulumElevationMap(
    loco::WholeBody& wholeBody, loco::TerrainModelElevationMap& terrain)
    : FootholdGeneratorInvertedPendulum(wholeBody, terrain) {

}


FootholdGeneratorInvertedPendulumElevationMap::~FootholdGeneratorInvertedPendulumElevationMap()
{

}

TerrainModelElevationMap& FootholdGeneratorInvertedPendulumElevationMap::getTerrainRef() {
  return static_cast<TerrainModelElevationMap&>(terrain_);
}

loco::Position FootholdGeneratorInvertedPendulumElevationMap::computeWorldToFootholdInWorldFrame(const int legId) {
  // fixme: remove get
  LegBase* leg = legs_.getPtr(legId);

  // Find starting point: hip projected vertically on ground.
  Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame();
  getTerrainRef().getHeight(positionWorldToHipOnPlaneAlongNormalInWorldFrame);

  // Evaluate the inverted pendulum (feedback) and velocity (feed-forward) contributions.
  const Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame = getPositionReferenceToDesiredFootOnTerrainInWorldFrame(*leg);

  // Build the desired foothold.
  Position positionWorldToFootholdInWorldFrame =
      positionWorldToHipOnPlaneAlongNormalInWorldFrame  // starting point, hip projected on the plane along world z axis
    + positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame;  // x-y
  getTerrainRef().getHeight(positionWorldToFootholdInWorldFrame);

  // Search for best foothold.
  const RotationQuaternion orientationControlToWorld = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().inverted();
  const LinearVelocity& commandVelocityInControlFrame = torso_.getDesiredState().getLinearVelocityTargetInControlFrame();
  const LinearVelocity commandVelocityInWorldFrame = orientationControlToWorld.rotate(commandVelocityInControlFrame);
  getTerrainRef().setCommandVelocity(commandVelocityInWorldFrame);
  Position positionWorldToSuitableFootholdInWorldFrame = getTerrainRef().getNearestSuitableFoothold(positionWorldToFootholdInWorldFrame, leg);

  leg->getFootPtr()->getStateDesiredPtr()->setPositionWorldToFootholdInWorldFrame(positionWorldToSuitableFootholdInWorldFrame);

  return positionWorldToSuitableFootholdInWorldFrame;
}


std::ostream& operator << (std::ostream& out, const FootholdGeneratorInvertedPendulum& fhGen) {
  out << std::endl;
  out << "----------------------------------------------------" << std::endl;
  out << "Class: FootholdGeneratorInvertedPendulumElevationMap" << std::endl;
  out << "step feedback scale: " << fhGen.getFeedbackScale() << std::endl;
  // fixme: remove get
  out << "default stepping position: " << std::endl
      << std::setw(10) << "left fore:"  << fhGen.getLegs().get(0).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl
      << std::setw(10) << "right fore:" << fhGen.getLegs().get(1).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl
      << std::setw(10) << "left hind:"  << fhGen.getLegs().get(2).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl
      << std::setw(10) << "right hind:" << fhGen.getLegs().get(3).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl;
  out << "----------------------------------------------------" << std::endl;
  out << std::endl;

  return out;
}

} /* namespace loco */
