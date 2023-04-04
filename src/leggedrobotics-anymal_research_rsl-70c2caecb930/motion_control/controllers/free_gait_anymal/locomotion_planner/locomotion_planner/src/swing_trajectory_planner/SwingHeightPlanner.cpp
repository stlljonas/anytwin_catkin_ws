/*
 * SwingHeightPlanner.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: Péter Fankhauser
 */

#include "locomotion_planner/swing_trajectory_planner/SwingHeightPlanner.hpp"
#include "locomotion_planner/common/geometry.hpp"
#include "locomotion_planner/common/type_defs.hpp"

namespace locomotion_planner {

SwingHeightPlanner::SwingHeightPlanner(const free_gait::AdapterBase& adapter, Parameters& parameters,
                                       std::shared_ptr<ElevationMapUser> elevationMapUser)
    : SwingTrajectoryPlannerBase(adapter, parameters),
      elevationMapUser_(elevationMapUser),
      stepCompleter_(stepParameters_, adapter),
      signedDistanceField_(nullptr)
{

}

SwingHeightPlanner::~SwingHeightPlanner()
{
}

bool SwingHeightPlanner::planSwingTrajectories(std::vector<free_gait::Step>& plan)
{
  if (!elevationMapUser_->isMapValid()) return false;
  signedDistanceField_ = &(elevationMapUser_->getSignedDistanceField(parameters_.getCollisionLayer()));
  bool success = true;
  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      free_gait::Step::LegMotions newLegMotions;
      for (auto& legMotion : step.getLegMotions()) {
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          if (!adapter_.isLegGrounded(legMotion.first)) {
            MELO_WARN("SwingHeightPlanner: Handling of this type of limb is not yet implemented!");
            continue;
          }
          free_gait::Footstep& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
          if (!planSwingHeight(footstep)) {
            MELO_WARN("SwingHeightPlanner: Could not plan swing height.");
            success = false;
            continue;
          }
        }
      }
    }
  }
  return success;
}

bool SwingHeightPlanner::planSwingHeight(free_gait::Footstep& footstep) const
{
  MELO_DEBUG_STREAM("Planning swing height for " << footstep.getLimb() << ".");

  // Increase height until collision free.
  const double maxHeightTriangle = 0.2;
  for (double height = footstep.getProfileHeight(); height < maxHeightTriangle; height += 0.01) {
    MELO_DEBUG_STREAM("Checking triangle swing trajectory with height " << height << " m for collision.");
    free_gait::Footstep footstepCopy(footstep);
    footstepCopy.setProfileType("triangle");
    footstepCopy.setProfileHeight(height);
    footstepCopy.updateStartPosition(adapter_.getPositionWorldToFootInWorldFrame(footstep.getLimb()));
    stepCompleter_.setParameters(footstepCopy);
    footstepCopy.compute(true);
    std::vector<std::pair<double, double>> collisions;
    if (!checkForCollision(footstepCopy, 0.06, collisions)) return false;
    bool collisionFree = true;
    for (const auto& collision : collisions) {
      const double phase = std::get<0>(collision) / footstepCopy.getDuration();
      if (phase > 0.15 && phase < 0.85) {
        if (std::get<1>(collision) < 0.04) {
          collisionFree = false;
          break;
        }
      }
    }
    if (collisionFree) {
      MELO_DEBUG_STREAM("Found collision-free trajectory.")
      footstep.setProfileType("triangle");
      footstep.setProfileHeight(height);
      return true;
    }
  }

  // Switch profile.
  for (double height = footstep.getProfileHeight(); height < 0.3; height += 0.01) {
    MELO_DEBUG_STREAM("Checking trapezoid swing trajectory with height " << height << " m for collision.");
    free_gait::Footstep footstepCopy(footstep);
    footstepCopy.setProfileType("trapezoid");
    footstepCopy.setProfileHeight(height);
    footstepCopy.updateStartPosition(adapter_.getPositionWorldToFootInWorldFrame(footstep.getLimb()));
    stepCompleter_.setParameters(footstepCopy);
    footstepCopy.compute(true);
    std::vector<std::pair<double, double>> collisions;
    if (!checkForCollision(footstepCopy, 0.06, collisions)) return false;
    bool collisionFree = true;
    for (const auto& collision : collisions) {
      const double phase = std::get<0>(collision) / footstepCopy.getDuration();
      if (phase > 0.1 && phase < 0.9) {
        if (std::get<1>(collision) < 0.04) {
          collisionFree = false;
          break;
        }
      }
    }
    if (collisionFree) {
      MELO_DEBUG("Found collision-free trajectory.")
      footstep.setProfileType("trapezoid");
      footstep.setProfileHeight(height);
      return true;
    }
  }

  // No collision free swing trajectory has been found, use triangle profile with max height.
  footstep.setProfileType("triangle");
  footstep.setProfileHeight(maxHeightTriangle);

  return false;
}

bool SwingHeightPlanner::checkForCollision(const free_gait::Footstep& footstep, const double margin,
                                           std::vector<std::pair<double, double>>& collisions) const
{
  MELO_DEBUG_STREAM("Checking for collisions.");
  collisions.clear();
  const double timeResolution = 0.03;
  if (!footstep.isComputed()) return false;
  for (double t = 0.0; t < footstep.getDuration(); t += timeResolution) {
    const Position position(footstep.evaluatePosition(t));
    const LinearVelocity velocity(footstep.evaluateVelocity(t));
    double distance = 0.0;
    if (signedDistanceField_) {
      MELO_DEBUG_STREAM("Checking SDF for collision at position " << position << ".");
      distance = signedDistanceField_->getDistanceAt(position.vector());
    } else {
      MELO_ERROR("SDF is not available.");
      return false;
    }
    if (distance < margin) {
      MELO_DEBUG_STREAM("Adding collision at time " << t << " s with distance " << distance << " m to list.");
      collisions.push_back(std::make_pair(t, distance));
    }
  }
  return true;
}

}
