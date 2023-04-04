/*
 * FootholdGeneratorInvertedPendulumMotionGen.hpp
 *
 *  Created on: April 12, 2018
 *      Author: Fabian Jenelten, C. Dario Bellicoso, Christian Gehring
 */

#pragma once

//loco.
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumBase.hpp"
#include "loco/foothold_generation/foothold_generator.hpp"

// motion_generation.
#include "motion_generation/ContactScheduleZmp.hpp"

namespace loco {

class FootholdGeneratorInvertedPendulumMotionGen : public FootholdGeneratorInvertedPendulumBase {
 public:
  FootholdGeneratorInvertedPendulumMotionGen(
      WholeBody& wholeBody,
      TerrainModelBase& terrain,
      ContactScheduleZmp& contactSchedule);

  ~FootholdGeneratorInvertedPendulumMotionGen() override = default;

  Position computeWorldToFootholdInWorldFrame(int legId) override;
  bool loadParameters(const TiXmlHandle& handle) override;

  //! Set default footholds w.r.t to base.
  void setDefaultFootholds(
      double distanceReferenceToDefaultFootholdHeading,
      double distanceReferenceToDefaultFootholdLateralHind,
      double distanceReferenceToDefaultFootholdLateralFront,
      double offsetHeading,
      double offsetLateral) override;

  LinearVelocity computeLinearVelocityProjectionInControlFrame(const LegBase& leg) const override;

  LinearVelocity computeLinearVelocityErrorInControlFrame(const LegBase& leg) const override;

 protected:
  //! Reference point to add velocity projection to obtain desired footholds.
  Position getPositionWorldToReferencePointInWorldFrame() const;

  Position evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg) override;
  Position evaluateDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg) override;

  //! Get desired foothold in world frame.
  Position getPositionReferenceToDesiredFootOnTerrainInWorldFrame(const LegBase& leg) override;

  //! A reference to the contact schedule.
  const ContactScheduleZmp& contactSchedule_;

  //! Position reference point (e.g.  COG or base) to default foothold in world frame.
  std_utils::EnumArray<contact_schedule::LegEnumAnymal, Position>positionReferencePointToDefaultFootholdInControlFrame_;

  //! Robot-centric foothold center.
  foothold_generator::FootholdCenter footholdCenter_;

  //! Feedback gain for inverted pendulum at a nominal height.
  double feedbackNominalInvertedPendulum_;

  //! If 1, robot (approximated as inverted pendulum), will align with gravity vector.
  //! If 0, robot will align with local terrain normal.
  //! If value is in (0,1), the projection vector will be linearly interpolated.
  double gravityFactor_;
};


} /* namespace loco */
