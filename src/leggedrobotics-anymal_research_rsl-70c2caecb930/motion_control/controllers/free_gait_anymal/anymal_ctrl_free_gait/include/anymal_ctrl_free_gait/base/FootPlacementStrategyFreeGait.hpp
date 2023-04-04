/*
 * FootPlacementStrategyFreeGait.hpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// Loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorDummy.hpp"

#include <loco_anymal/common/LegsAnymal.hpp>

// Anymal model
#include "anymal_model/AnymalModel.hpp"

// TinyXML
#include "tinyxml.h"

// Eigen
#include <Eigen/Core>

// Kindr
#include "kindr/Core"

// STD
#include <vector>

#include "robot_utils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp"

namespace loco {

class FootPlacementStrategyFreeGait : public FootPlacementStrategyBase {
 private:
  using AD = anymal_description::AnymalDescription;

 public:
  FootPlacementStrategyFreeGait(loco_anymal::LegsAnymal& legs, TorsoBase& torso, anymal_model::AnymalModel& anymalModel,
                                anymal_model::AnymalModel& anymalModelDesired, TerrainModelBase& terrain, WholeBody& wholeBody,
                                free_gait::Executor& executor);
  ~FootPlacementStrategyFreeGait() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool initialize(double dt) override;
  bool advance(double dt) override;

  void updateLegConfigurations();

  void generateDesiredLegMotion(LegBase* leg);

  void regainContact(LegBase* leg, const double dt);

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to footPlacementStrategy1, 1 -> footPlacementStrategy2, and values in between
   *  correspond to interpolated parameter set.
   * @param footPlacementStrategy1
   * @param footPlacementStrategy2
   * @param t interpolation parameter
   * @returns true if successful
   */
  bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2,
                         double t) override;
  const loco_anymal::LegsAnymal& getLegs() const;

  const SwingTrajectoryGeneratorBase& getSwingTrajectoryGenerator() const override;
  SwingTrajectoryGeneratorBase* getSwingTrajectoryGeneratorPtr() override;

 protected:
  //! Legs information container.
  loco_anymal::LegsAnymal& legs_;

  //! Torso information container.
  TorsoBase& torso_;

  //! Robot model.
  anymal_model::AnymalModel& anymalModel_;
  anymal_model::AnymalModel& anymalModelDesired_;

  //! Terrain model information container.
  TerrainModelBase& terrain_;

  WholeBody& wholeBody_;

  //! Free gait executor.
  free_gait::Executor& executor_;

  //! Two step phases of the regain contact sequence.
  std::unordered_map<AD::LimbEnum, bool, free_gait::EnumClassHash> isLegInRegainMode_;

  //! For regaining contact, how far leg should be stretched.
  double maxRegainContactDistance_;

  //! Speed at end of regain contact.
  double endSpeedRegainContact_;

  //! A dummy trajectory generator.
  SwingTrajectoryGeneratorDummy swingTrajectoryGenerator_;
};

} /* namespace loco */
