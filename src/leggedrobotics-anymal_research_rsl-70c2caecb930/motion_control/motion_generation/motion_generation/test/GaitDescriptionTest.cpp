/*!
* @file     zmpOptTest.cpp
* @author   Fabian Jenelten
* @date     Feb. 08, 2018
* @brief
*/

// gtest
#include <gtest/gtest.h>

// motion_generation
#include "motion_generation/GaitDescription.hpp"

// robot utils
#include "robot_utils/math/math.hpp"



TEST(GaitDescription, flying_trot) {
  using namespace loco;
  bool success = true;

  std::vector<contact_schedule::LegEnumAnymal> legEnumsUsedForWalk;
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::LF);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::RF);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::LH);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::RH);

  contact_schedule::periodic::GaitDescription flyingTrot;

  // Construct gait.
  flyingTrot.clear();
  flyingTrot.setNominalStrideDuration(0.52);
  flyingTrot.push_back_phases(contact_schedule::LegEnumAnymal::LF, 0.0, 0.64);
  flyingTrot.push_back_phases(contact_schedule::LegEnumAnymal::RF, 0.5, 1.14);
  flyingTrot.push_back_phases(contact_schedule::LegEnumAnymal::LH, 0.5, 1.14);
  flyingTrot.push_back_phases(contact_schedule::LegEnumAnymal::RH, 0.0, 0.64);
  success &= flyingTrot.createEventContainer();

  // Check gait for inconsistencies.
  success &= flyingTrot.getEventContainer().checkGait(legEnumsUsedForWalk);

  // Check for event detection.
  bool shouldLegBeGrounded; double phaseUntilNextLiftOff; double phaseUntilNextTouchDown;

  flyingTrot.getGaitDescriptionForLeg(contact_schedule::LegEnumAnymal::RH).getPhaseUntilNextEvents(
      shouldLegBeGrounded, phaseUntilNextLiftOff, phaseUntilNextTouchDown, 0.9);
  success &= (
      shouldLegBeGrounded &&
      robot_utils::areNear(phaseUntilNextLiftOff, 0.1) &&
      robot_utils::areNear(phaseUntilNextTouchDown, 0.74)
  );

  flyingTrot.getGaitDescriptionForLeg(contact_schedule::LegEnumAnymal::RF).getPhaseUntilNextEvents(
      shouldLegBeGrounded, phaseUntilNextLiftOff, phaseUntilNextTouchDown, 0.5);
  success &= (
      !shouldLegBeGrounded &&
      robot_utils::areNear(phaseUntilNextLiftOff, 1.0) &&
      robot_utils::areNear(phaseUntilNextTouchDown, 0.64)
  );

  // Check event container.
  const double fullFlightPhase =  flyingTrot.getEventContainer().findLargestFullFlightPhase(legEnumsUsedForWalk);
  const double fullStancePhase = flyingTrot.getEventContainer().findLargestFullStancePhase(legEnumsUsedForWalk);
  const unsigned int numOfSwingingLegsAtStart = flyingTrot.getEventContainer().numOfSwingingLegsAtStart(legEnumsUsedForWalk);

  success &= (
      robot_utils::areNear(fullFlightPhase, 0.14) &&
      robot_utils::areNear(fullStancePhase, -1.0) &&
      numOfSwingingLegsAtStart == 2u
  );

  std::vector<contact_schedule::LegEnumAnymal> nextLifOffLegs; double phase;
  success &= flyingTrot.getEventContainer().findNextLiftOffLegs(contact_schedule::LegEnumAnymal::LF, nextLifOffLegs, phase);

  success &= (
      nextLifOffLegs.size() == 2 &&
      std::count(nextLifOffLegs.begin(), nextLifOffLegs.end(), contact_schedule::LegEnumAnymal::LH) == 1 &&
      std::count(nextLifOffLegs.begin(), nextLifOffLegs.end(), contact_schedule::LegEnumAnymal::RF) == 1 &&
      robot_utils::areNear(phase, 0.5)
  );

  EXPECT_TRUE(success);
}

TEST(GaitDescription, trot) {
  using namespace loco;
  bool success = true;

  std::vector<contact_schedule::LegEnumAnymal> legEnumsUsedForWalk;
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::LF);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::RF);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::LH);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::RH);

  contact_schedule::periodic::GaitDescription trot;

  // Construct gait.
  trot.clear();
  trot.setNominalStrideDuration(0.85);
  trot.push_back_phases(contact_schedule::LegEnumAnymal::LF, 0.02, 0.48);
  trot.push_back_phases(contact_schedule::LegEnumAnymal::RF, 0.52, 0.98);
  trot.push_back_phases(contact_schedule::LegEnumAnymal::LH, 0.52, 0.98);
  trot.push_back_phases(contact_schedule::LegEnumAnymal::RH, 0.02, 0.48);
  success &= trot.createEventContainer();

  // Check gait for inconsistencies.
  success &= trot.getEventContainer().checkGait(legEnumsUsedForWalk);

  // Check event container.
  const double fullFlightPhase =  trot.getEventContainer().findLargestFullFlightPhase(legEnumsUsedForWalk);
  const double fullStancePhase = trot.getEventContainer().findLargestFullStancePhase(legEnumsUsedForWalk);
  const unsigned int numOfSwingingLegsAtStart = trot.getEventContainer().numOfSwingingLegsAtStart(legEnumsUsedForWalk);

  success &= (
      robot_utils::areNear(fullFlightPhase, -1.0) &&
      robot_utils::areNear(fullStancePhase, 0.04) &&
      numOfSwingingLegsAtStart == 0u
  );

  EXPECT_TRUE(success);
}

TEST(GaitDescription, crawl_no_stance_phase) {
  using namespace loco;
  bool success = true;

  std::vector<contact_schedule::LegEnumAnymal> legEnumsUsedForWalk;
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::LF);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::RF);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::LH);
  legEnumsUsedForWalk.push_back(contact_schedule::LegEnumAnymal::RH);

  contact_schedule::periodic::GaitDescription crawl_no_stance_phase;
  // Construct gait.
  crawl_no_stance_phase.clear();
  crawl_no_stance_phase.setNominalStrideDuration(1.4);
  crawl_no_stance_phase.push_back_phases(contact_schedule::LegEnumAnymal::LF, 0.00, 0.25);
  crawl_no_stance_phase.push_back_phases(contact_schedule::LegEnumAnymal::RF, 0.25, 0.50);
  crawl_no_stance_phase.push_back_phases(contact_schedule::LegEnumAnymal::LH, 0.50, 0.75);
  crawl_no_stance_phase.push_back_phases(contact_schedule::LegEnumAnymal::RH, 0.75, 1.00);
  success &= crawl_no_stance_phase.createEventContainer();

  // Check gait for inconsistencies.
  success &= crawl_no_stance_phase.getEventContainer().checkGait(legEnumsUsedForWalk);

  // Check event container.
  const double fullFlightPhase =  crawl_no_stance_phase.getEventContainer().findLargestFullFlightPhase(legEnumsUsedForWalk);
  const double fullStancePhase = crawl_no_stance_phase.getEventContainer().findLargestFullStancePhase(legEnumsUsedForWalk);
  const unsigned int numOfSwingingLegsAtStart = crawl_no_stance_phase.getEventContainer().numOfSwingingLegsAtStart(legEnumsUsedForWalk);

  success &= (
      robot_utils::areNear(fullFlightPhase, -1.0) &&
      robot_utils::areNear(fullStancePhase, -1.0) &&
      numOfSwingingLegsAtStart == 1u
  );

  EXPECT_TRUE(success);
}


