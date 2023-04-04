/*!
 * @file 	FootPlacementStrategyBase.hpp
 * @author 	Christian Gehring, Stelian Coros
 * @date		  Sep 7, 2012
 * @version 	1.0
 * @ingroup
 * @brief
 */

#pragma once

// stl
#include <memory>
#include <vector>

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/typedefs.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp"

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {

/*!
 * A foot placement strategy module calls both a foothold generator and a swing foot trajectory generator, then forwards their respective
 * targets to the corresponding desired end-effector states.
 *
 * Inputs:
 * - for each leg, ``leg->foot->stateDesired->getPositionWorldToFootholdInWorldFrame()``
 * - ``torso->measuredState->inControlFrame->getOrientationWorldToControl()``
 *
 * Outputs:
 * - for each ``limb->endEffector->stateDesired``:
 *   - setPositionWorldToEndEffectorInWorldFrame()
 *   - setLinearVelocityEndEffectorInWorldFrame()
 *
 * Advanced by: main controller module.
 *
 */
class FootPlacementStrategyBase : public ModuleBase {
 public:
  FootPlacementStrategyBase();
  ~FootPlacementStrategyBase() override = default;

  virtual bool goToStand();
  virtual bool resumeWalking();

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to footPlacementStrategy1, 1 -> footPlacementStrategy2, and values in between
   *  correspond to interpolated parameter set.
   * @param footPlacementStrategy1
   * @param footPlacementStrategy2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1,
                                 const FootPlacementStrategyBase& footPlacementStrategy2, double t);

  virtual const SwingTrajectoryGeneratorBase& getSwingTrajectoryGenerator() const = 0;
  virtual SwingTrajectoryGeneratorBase* getSwingTrajectoryGeneratorPtr() = 0;

 protected:
  bool isFirstTimeInit_;
};

}  // namespace loco
