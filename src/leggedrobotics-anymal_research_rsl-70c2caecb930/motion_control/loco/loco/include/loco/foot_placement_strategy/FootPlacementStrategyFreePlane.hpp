/*!
 * @file     FootPlacementStrategyFreePlane.hpp
 * @author   C. Dario Bellicoso, Christian Gehring
 * @date     Sep 16, 2014
 * @brief
 */
#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/foothold_generation/FootholdGeneratorFreePlane.hpp"
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumBase.hpp"
#include "loco/gait_pattern/ContactScheduleLock.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp"

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {

//! Foot placement strategy for the free plane
/*! This strategy is used for the trotting gait.
 *
 */
class FootPlacementStrategyFreePlane : public FootPlacementStrategyBase {
 public:
  FootPlacementStrategyFreePlane(WholeBody& wholeBody, TerrainModelBase& terrain, ContactScheduleLock& contactSchedule,
                                 SwingTrajectoryGeneratorBase& swingTrajectoryGenerator,
                                 FootholdGeneratorInvertedPendulumBase& footholdGenerator);
  ~FootPlacementStrategyFreePlane() override = default;

  bool advance(double dt) override;
  bool initialize(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;

  const std::vector<Position>& getPositionWorldToDesiredFootHoldInWorldFrame() const;
  const std::vector<Position>& getPositionWorldToDesiredFootInWorldFrame() const;
  const std::vector<Position>& getPositionWorldToHipOnPlaneAlongNormalInWorldFrame() const;
  const std::vector<Position>& getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame() const;
  const std::vector<Position>& getPositionDesiredFootOnTerrainToDesiredFootInWorldFrame() const;
  const std::vector<Position>& getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame() const;
  const std::vector<Position>& getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame() const;
  const std::vector<Position>& getPositionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame() const;

  const std::vector<Position>& getPositionWorldToDefaultFootHoldInWorldFrame() const;
  const std::vector<Position>& getPositionWorldToFootHoldInvertedPendulumInWorldFrame() const;

  const Legs& getLegs() const;

  bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2,
                         double t) override;

  const FootholdGeneratorInvertedPendulumBase& getFootholdGenerator() const;
  FootholdGeneratorInvertedPendulumBase* getFootholdGeneratorPtr();

  const SwingTrajectoryGeneratorBase& getSwingTrajectoryGenerator() const override;
  SwingTrajectoryGeneratorBase* getSwingTrajectoryGeneratorPtr() override;

 protected:
  /*! Compute and return the current desired foot position in world frame.
   * @params[in] leg The leg relative to the desired foot.
   * @returns The desired foot position in world frame.
   */
  void computeDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame, LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                               LinearAcceleration& linearAccelerationDesiredFootInWorldFrame, LegBase* leg, double dt);
  /*! Project a point on a plane along the plane's normal.
   * @params[in] position The coordinates of the point that has to be projected.
   * @returns The coordinates of the point projected on the terrain along the surface normal.
   */
  virtual Position getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position);

  virtual void setFootTrajectory(LegBase* leg, double dt);
  virtual void regainContact(LegBase* leg, double dt);

  //! Reference to the torso.
  TorsoBase& torso_;

  //! Velocity for regain contact
  double regainVelocity_;

  //! Velocity for interpolation between footholds
  double maxFootholdDisplacementVelocity_;

  //! Reference to the legs.
  Legs& legs_;

  //! Reference to the terrain.
  TerrainModelBase& terrain_;

  //! Reference to the gait pattern
  ContactScheduleLock& contactSchedule_;

  //! A reference to the swing trajectory planner.
  SwingTrajectoryGeneratorBase& swingTrajectoryGenerator_;

  //! A reference to the foothold planner.
  FootholdGeneratorInvertedPendulumBase& footholdGenerator_;

  //! Desired foothold and desired foot position
  std::vector<Position> positionWorldToOldInterpolatedFootHoldInWorldFrame_;
  std::vector<Position> positionWorldToOldDesiredFootHoldInWorldFrame_;
  std::vector<Position> positionWorldToDesiredFootHoldInWorldFrame_;
  std::vector<Position> positionWorldToDesiredFootInWorldFrame_;

  std::vector<Position> positionWorldToHipOnPlaneAlongNormalInWorldFrame_;
  std::vector<Position> positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_;
  std::vector<Position> positionDesiredFootOnTerrainToDesiredFootInWorldFrame_;
  std::vector<Position> positionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame_;
  std::vector<Position> positionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame_;
  std::vector<Position> positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_;

  std::vector<Position> positionWorldToDefaultFootHoldInWorldFrame_;
  std::vector<Position> positionWorldToFootHoldInvertedPendulumInWorldFrame_;
};

} /* namespace loco */
