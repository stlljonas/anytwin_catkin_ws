/*!
* @file     FootPlacementStrategyOptimized.hpp
* @author   Fabian Jenelten
* @date     Mar 20, 2018
* @brief
*/

#pragma once

// loco.
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumBase.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorModule.hpp"

// motion_generation_loco.
#include "loco/foot_placement_strategy/foot_placement_strategy.hpp"
#include "loco/foothold_generation/InvPendParameterHandler.hpp"
#include "loco/planner/ModulePlanner.hpp"
#include "loco/torso_control/TerrainAdapter.hpp"

// motion generation.
#include "motion_generation/ContactScheduleZmp.hpp"

class TiXmlHandle;

namespace loco {

template<typename FootholdOptimizer_, typename FootholdPlan_>
class FootPlacementStrategyOptimized: public FootPlacementStrategyBase {
 public:
  FootPlacementStrategyOptimized(
      WholeBody& wholeBody,
      TerrainModelBase& terrain,
      SwingTrajectoryGeneratorModule& swingTrajectoryGenerator,
      FootholdOptimizer_& footholdGenerator,
      ContactScheduleZmp& contactSchedule,
      HeadingGenerator& headingGenerator,
      const TerrainAdapter& terrainAdapter,
      FootholdGeneratorInvertedPendulumBase& referenceFootholdGenerator);

  ~FootPlacementStrategyOptimized() override = default;

  bool advance(double dt) override;
  bool initialize(double dt) override;
  bool stop() override;
  bool loadParameters(const TiXmlHandle& handle) override;

  const Legs& getLegs() const;

  virtual bool setToInterpolated(
        const FootPlacementStrategyBase& footPlacementStrategy1,
        const FootPlacementStrategyBase& footPlacementStrategy2, double t);

  const SwingTrajectoryGeneratorBase& getSwingTrajectoryGenerator() const override;
  SwingTrajectoryGeneratorBase* getSwingTrajectoryGeneratorPtr() override;

  void getFootholdPlan(FootholdPlan_& plan) const;

  //! Scales regain position with stride duration of the current gait.
  double computeRegainVelocity() const;

 protected:

  /*! Compute and return the current desired foot position in world frame.
   * @params[in] leg The leg relative to the desired foot.
   * @returns The desired foot position in world frame.
   */
  void computeDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame,
                               LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                               LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                               LegBase* leg,
                               double dt);
  /*! Project a point on a plane along the plane's normal.
   * @params[in] position The coordinates of the point that has to be projected.
   * @returns The coordinates of the point projected on the terrain along the surface normal.
   */
  virtual Position getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position);

  virtual void setFootTrajectory(LegBase* leg, double dt);
  virtual void regainContact(LegBase* leg, double dt);
  virtual void setFootOnGround(LegBase* leg, double dt);

  void setDesiredFootState(
      LegBase* leg,
      const Position& positionWorldToFootInWorldFrame,
      const LinearVelocity& linearVelocityDesFootInWorldFrame,
      const LinearAcceleration& linearAccelerationDesFootInWorldFrame);

  SwingTrajectoryGeneratorModule* getSwingTrajectoryGeneratorModulePtr() {
    return static_cast<SwingTrajectoryGeneratorModule*>(&swingTrajectoryGenerator_);
  }

  virtual bool setFootholdPlan();

  //! Updates singularity value for leg.
  virtual void updateSingularityValue(LegBase* leg);

  //! Reference to the torso.
  WholeBody& wholeBody_;

  //! Reference to the torso.
  TorsoBase& torso_;

  //! Reference to the legs.
  Legs& legs_;

  //! Reference to the terrain.
  TerrainModelBase& terrain_;

  //! Reference to some other (static) foothold generator.
  FootholdGeneratorInvertedPendulumBase& referenceFootholdGenerator_;

  //! A reference to the swing trajectory planner.
  SwingTrajectoryGeneratorBase& swingTrajectoryGenerator_;

  //! The foothold planner.
  ModulePlanner<FootholdOptimizer_, FootholdPlan_> footholdPlanner_;

  //! A foothold plan container.
  FootholdPlan_ plan_;

  //! A reference to the gait pattern.
  ContactScheduleZmp& contactSchedule_;

  //! A reference to the heading generator.
  const HeadingGenerator& headingGenerator_;

  //! Reference to the terrain adapter used to update footstep plans.
  const TerrainAdapter& terrainAdapter_;

  //! A mutex to synchronize access to the motion plan.
  mutable boost::shared_mutex mutexFootholdPlan_;

  //! Regain distance (regain velocity times stride duration of the gait).
  double regainPosition_;

  // Parameters for inverted pendulum foothold generator.
  foothold_generator::InvPendParameterHandler invPendParamHandler_;

  //! Value indicating over-extension (0: completely contracted, 1: leg over-extension).
  std_utils::EnumArray<contact_schedule::LegEnumAnymal, double> singularityValue_;

  //! Singularity value when entering contact recovery.
  std_utils::EnumArray<contact_schedule::LegEnumAnymal, double> singularityValueAtStartOfContactRecovery_;

  //! Strategy for impedance control while slipping.
  fps::SlippageRecoveryStrategy slippageRecoveryStrategy_;
};

} /* namespace loco */

#include <loco/foot_placement_strategy/FootPlacementStrategyOptimized.tpp>

