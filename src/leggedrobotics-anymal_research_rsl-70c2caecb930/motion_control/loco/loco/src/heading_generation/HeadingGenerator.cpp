/*
 * HeadingGenerator.cpp
 *
 *  Created on: Jan 26, 2017
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/heading_generation/HeadingGenerator.hpp"

namespace loco {

void HeadingGenerator::getOrientationWorldToTorsoHeading(loco::RotationQuaternion& orientationWorldToHeading) const {
  Vector vectorHeadingDirectionInWorldFrame;
  getTorsoHeadingDirectionInWorldFrame(vectorHeadingDirectionInWorldFrame);
  vectorHeadingDirectionInWorldFrame.z() = 0.0;

  try {
    orientationWorldToHeading.setFromVectors(vectorHeadingDirectionInWorldFrame.toImplementation(), Vector::UnitX().toImplementation());
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in HeadingGenerator::getOrientationWorldToTorsoHeading().")
    orientationWorldToHeading.setIdentity();
  }
}

void HeadingGenerator::getOrientationWorldToPreviousFootprintHeading(loco::RotationQuaternion& orientationWorldToHeading) const {
  Vector vectorHeadingDirectionInWorldFrame;
  getLegsHeadingDirectionFromLastContactsInWorldFrame(vectorHeadingDirectionInWorldFrame);
  vectorHeadingDirectionInWorldFrame.z() = 0.0;

  try {
    orientationWorldToHeading.setFromVectors(vectorHeadingDirectionInWorldFrame.toImplementation(), Vector::UnitX().toImplementation());
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in HeadingGenerator::getOrientationWorldToPreviousFootprintHeading().")
    orientationWorldToHeading.setIdentity();
  }
}

void HeadingGenerator::getOrientationWorldToCurrentFootprintHeading(loco::RotationQuaternion& orientationWorldToHeading) const {
  Vector vectorHeadingDirectionInWorldFrame;
  getLegsHeadingDirectionFromCurrentFeetInWorldFrame(vectorHeadingDirectionInWorldFrame);
  vectorHeadingDirectionInWorldFrame.z() = 0.0;

  try {
    orientationWorldToHeading.setFromVectors(vectorHeadingDirectionInWorldFrame.toImplementation(), Vector::UnitX().toImplementation());
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in HeadingGenerator::getOrientationWorldToCurrentFootprintHeading().")
    orientationWorldToHeading.setIdentity();
  }
}

void HeadingGenerator::getOrientationWorldToPlannedFootprintHeading(loco::RotationQuaternion& orientationWorldToHeading) const {
  Vector vectorHeadingDirectionInWorldFrame;
  getLegsHeadingDirectionFromPlannedContactsInWorldFrame(vectorHeadingDirectionInWorldFrame);
  vectorHeadingDirectionInWorldFrame.z() = 0.0;

  try {
    orientationWorldToHeading.setFromVectors(vectorHeadingDirectionInWorldFrame.toImplementation(), Vector::UnitX().toImplementation());
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in HeadingGenerator::getOrientationWorldToPlannedFootprintHeading().")
    orientationWorldToHeading.setIdentity();
  }
}

loco::Pose HeadingGenerator::getPoseLastFootPrintCenterToWorld() const {
  loco::Position positionWorldToFootPrintCenterInWorldFrame;
  computeLastFootPrintCenterInWorldFrame(positionWorldToFootPrintCenterInWorldFrame);

  loco::Vector headingDirectionInWorldFrame;
  getLegsHeadingDirectionFromLastContactsInWorldFrame(headingDirectionInWorldFrame);
  headingDirectionInWorldFrame.z() = 0.0;
  loco::RotationQuaternion orientationFootPrintCenterToWorld;
  try {
    orientationFootPrintCenterToWorld.setFromVectors(loco::Vector::UnitX().toImplementation(),
                                                     headingDirectionInWorldFrame.toImplementation());
  } catch (const std::runtime_error& error) {
    MELO_WARN_STREAM(error.what() << "setFromVectors in HeadingGenerator::getPoseLastFootPrintCenterToWorld().")
    orientationFootPrintCenterToWorld.setIdentity();
  }

  return loco::Pose{positionWorldToFootPrintCenterInWorldFrame, orientationFootPrintCenterToWorld};
}

} /* namespace loco */
