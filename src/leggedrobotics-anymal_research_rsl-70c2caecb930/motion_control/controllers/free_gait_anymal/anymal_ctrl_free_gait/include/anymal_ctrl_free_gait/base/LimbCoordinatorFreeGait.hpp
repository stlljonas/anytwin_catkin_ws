/*
 * LimbCoordinatorFreeGait.hpp
 *
 *  Created on: Jun 17, 2015
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <loco_anymal/common/LegsAnymal.hpp>
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

// STD
#include <memory>

namespace loco {

class LimbCoordinatorFreeGait : public LimbCoordinatorBase {
 public:
  LimbCoordinatorFreeGait(loco_anymal::LegsAnymal& legs, TorsoBase& torso, free_gait::Executor& executor, GaitPatternBase& gaitPattern);
  ~LimbCoordinatorFreeGait() override = default;

  bool initialize(double dt) override;

  /*! Advance in time
   * @param dt  time step [s]
   */
  bool advance(double dt) override;

  //  std::shared_ptr<GaitPatternBase> getGaitPattern();
  GaitPatternBase* getGaitPatternPtr() override;  // Deprecated, do not use.
  const GaitPatternBase& getGaitPattern() const;

  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to limbCoordinator1, 1 -> limbCoordinator2, and values in between
   *  correspond to interpolated parameter set.
   * @param limbCoordinator1
   * @param limbCoordinator2
   * @param t interpolation parameter
   * @returns true if successful
   */
  bool setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double t) override;

 protected:
  loco_anymal::LegsAnymal& legs_;
  TorsoBase& torso_;
  free_gait::Executor& executor_;
  GaitPatternBase& gaitPattern_;
};

} /* namespace loco */
