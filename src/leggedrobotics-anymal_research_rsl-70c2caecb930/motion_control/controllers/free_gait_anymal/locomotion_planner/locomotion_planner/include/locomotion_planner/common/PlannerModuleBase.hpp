/*
 * PlannerModuleBase.hpp
 *
 *  Created on: Mar 7, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "type_defs.hpp"

#pragma once

namespace locomotion_planner {

class PlannerModuleBase
{
 public:
  PlannerModuleBase();
  virtual ~PlannerModuleBase();

  //! Compute the plan.
  //! @param[in] startPose the desired start pose.
  //! @param[in] goalPose the desired goal pose.
  //! @param[in/out] plan initial information and the plan updates filled in.
  //! @return true if planning was successful, false otherwise.
  virtual bool plan(const Pose& goalPose, const Transform& goalToWorldFrameTransform, const double speedFactor,
                    const bool isNewGoal, const bool allowIntermediatePoses, std::vector<Step>& plan,
                    bool& lastStepToGoal) = 0;

  //! @return the expected upper planning time (might be exceeded in special circumstances).
  virtual double getExpectedUpperPlanningTime() const = 0;

  //! Stop planning.
  virtual void stop() = 0;
};

} /* namespace locomotion_planner */
