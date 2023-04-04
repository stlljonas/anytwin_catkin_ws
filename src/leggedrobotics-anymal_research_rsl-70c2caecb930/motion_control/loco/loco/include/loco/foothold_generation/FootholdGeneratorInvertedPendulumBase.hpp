/*
 * FootholdGeneratorInvertedPendulumBase.hpp
 *
 *  Created on: April 12, 2018
 *      Author: Fabian Jenelten, C. Dario Bellicoso, Christian Gehring
 */

#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/foothold_generation/FootholdGeneratorFreePlane.hpp"

namespace loco {

class FootholdGeneratorInvertedPendulumBase : public FootholdGeneratorFreePlane {
 public:
  FootholdGeneratorInvertedPendulumBase(WholeBody& wholeBody, TerrainModelBase& terrain);
  ~FootholdGeneratorInvertedPendulumBase() override = default;

  //! Set velocity feedback gain.
  void setFeedbackScale(double feedbackScale) noexcept;

  //! Get velocity feedback gain.
  double getFeedbackScale() const noexcept;

  //! Set velocity projection feedforward gain.
  void setFeedforwardScale(double feedforwardScale) noexcept;

  //! Get velocity projection feedforward gain.
  double getFeedforwardScale() const noexcept;

  //! Set default footholds w.r.t to base.
  virtual void setDefaultFootholds(double distanceReferenceToDefaultFootholdHeading, double distanceReferenceToDefaultFootholdLateralHind,
                                   double distanceReferenceToDefaultFootholdLateralFront, double offsetHeading, double offsetLateral);

  const Legs& getLegs() const;

  //! Get foothold correction (incorporates the feedback term).
  const Position& getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame(int legId) const override;

  //! Get desired foothold based on velocity projection (incorporates the feedforward).
  const Position& getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame(int legId) const override;

  //! Get linear velocity feedforward projection in control frame (based on desired base velocity).
  virtual LinearVelocity computeLinearVelocityProjectionInControlFrame(const LegBase& leg) const = 0;

  //! Get linear velocity error in control frame (based on base and limb base velocity).
  virtual LinearVelocity computeLinearVelocityErrorInControlFrame(const LegBase& leg) const = 0;

 protected:
  //! Get desired foothold in world frame.
  virtual Position getPositionReferenceToDesiredFootOnTerrainInWorldFrame(const LegBase& leg);

  //! Feedback position vector is scaled with the following gain (and additionally with stepFeedbackScale_).
  virtual double getFeedbackInvertedPendulum(const LegBase& leg) const;

  //! Compute velocity projection foothold.
  virtual Position evaluateDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg) = 0;

  //! Compute feedback correction.
  virtual Position evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg) = 0;

  //! Default foothold expressed in world frame.
  virtual Position getPositionWorldToDefaultFootholdOnGroundInWorldFrame(const LegBase& leg) const;

  //! A const reference to the whole body.
  const WholeBody& wholeBody_;

  //! A reference to the torso.
  TorsoBase& torso_;

  //! A reference to the legs.
  Legs& legs_;

  //! A reference to the terrain.
  TerrainModelBase& terrain_;

  //! Gain for velocity feedback.
  double stepFeedbackScale_;

  //! Gain for velocity projection feedforward.
  double stepFeedforwardScale_;

  //! Foothold correction (incorporates the feedback term).
  std::vector<Position> positionDesiredFootHoldOnTerrainFeedBackInControlFrame_;

  //! Desired foothold based on velocity projection (incorporates the feedforward).
  std::vector<Position> positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_;
};

} /* namespace loco */
