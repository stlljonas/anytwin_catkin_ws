/*
 * FootprintGenerator.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Dario Bellicoso, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// loco
#include <loco/common/FootprintGenerator.hpp>

namespace loco {

FootprintGenerator::FootprintGenerator() : footprintPose_(), headingDirectionInWorldFrame_() {}

bool FootprintGenerator::update(const Legs& legs, const HeadingGenerator& headingGenerator) {
  // Reset the footprint pose.
  footprintPose_.setIdentity();

  // Find footprint origin.
  for (const auto& leg : legs) {
    footprintPose_.getPosition() += leg->getPositionWorldToLastOrCurrentContactInWorldFrame();
  }
  if (legs.size() > 0u) {
    footprintPose_.getPosition() /= legs.size();
  }

  // Find footprint orientation.
  headingGenerator.getLegsHeadingDirectionFromLastContactsInWorldFrame(headingDirectionInWorldFrame_);
  headingDirectionInWorldFrame_.z() = 0.0;

  try {
    footprintPose_.getRotation().setFromVectors(loco::Vector::UnitX().toImplementation(), headingDirectionInWorldFrame_.toImplementation());
  } catch (const std::runtime_error& error) {
    MELO_WARN("[FootprintGeneratorAnymal::update] Caught kindr setFromVectors() exception!");
    return false;
  }
  return true;
}

const loco::Pose& FootprintGenerator::getFootprintPose() const {
  return footprintPose_;
}

const loco::Vector& FootprintGenerator::getFootprintHeadingDirectionInWorldFrame() const {
  return headingDirectionInWorldFrame_;
}

} /* namespace loco */
