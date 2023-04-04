/*
 * Geometry.cpp
 *
 *  Created on: Mar 14, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "locomotion_planner/common/geometry.hpp"

namespace locomotion_planner {

const Position2 getPlanarPositionFromPosition(const Position& position)
{
  Position2 planarPosition;
  planarPosition(0) = position.x();
  planarPosition(1) = position.y();
  return planarPosition;
}

const Position getPositionFromPlanarPosition(const Position2& planarPosition)
{
  Position position;
  position.x() = planarPosition(0);
  position.y() = planarPosition(1);
  position.z() = 0.0;
  return position;
}

const Pose getPoseFromPlanarPose(const PlanarPose& planarPose)
{
  Pose pose;
  pose.getPosition().x() = planarPose.x();
  pose.getPosition().y() = planarPose.y();
  pose.getPosition().z() = 0.0;
  pose.getRotation() = RotationQuaternion(RotationVector(0.0, 0.0, planarPose.z()));
  return pose;
}

const PlanarPose getPlanarPoseFromPose(const Pose& pose)
{
  PlanarPose planarPose;
  planarPose.x() = pose.getPosition().x();
  planarPose.y() = pose.getPosition().y();
  planarPose.z() = RotationVector(pose.getRotation()).z();
  return planarPose;
}

const PlanarTwist getPlanarTwistFromTwist(const Twist& twist)
{
  PlanarTwist planarTwist;
  planarTwist.x() = twist.getTranslationalVelocity().x();
  planarTwist.y() = twist.getTranslationalVelocity().y();
  planarTwist.z() = twist.getRotationalVelocity().z();
  return planarTwist;
}

const Pose getFootprintPoseFromStance(const Stance& stance)
{
  Pose footprint;

  // Position.
  for (const auto& leg : stance) {
    footprint.getPosition() += leg.second;
  }
  if (stance.size() > 0u) {
    footprint.getPosition() /= stance.size();
  }

  // Orientation.
  const Position foreFeetMidPoint = (stance.at(LimbEnum::LF_LEG) + stance.at(LimbEnum::RF_LEG)) * 0.5;
  const Position hindFeetMidPoint = (stance.at(LimbEnum::LH_LEG) + stance.at(LimbEnum::RH_LEG)) * 0.5;
  Vector headingDirection((foreFeetMidPoint - hindFeetMidPoint).vector());
  headingDirection.z() = 0.0;
  footprint.getRotation().setFromVectors(Vector::UnitX().toImplementation(), headingDirection.toImplementation());
  return footprint;
}

const Pose getNextFootprintPose(const free_gait::AdapterBase& adapter, const free_gait::StepQueue& plan)
{
  Stance stance;
  for (const auto& limb : adapter.getLimbs()) {
    stance[limb] = getFootPositionAfterActiveStep(limb, adapter, plan);
  }
  return getFootprintPoseFromStance(stance);
}

const Position getFootPositionAfterActiveStep(const LimbEnum& limb, const free_gait::AdapterBase& adapter, const free_gait::StepQueue& plan)
{
  Position footPositionInWorld;
  if (adapter.isLegGrounded(limb)) {
    footPositionInWorld = adapter.getPositionWorldToFootInWorldFrame(limb);
  } else {
    bool isPlannedFor = false;
    for (const auto& step : plan.getQueue()) {
      if (step.hasLegMotion(limb)) {
        if (step.getLegMotion(limb).getTrajectoryType() == free_gait::LegMotionBase::TrajectoryType::EndEffector) {
          const auto& endEffectorMotion = static_cast<const free_gait::EndEffectorMotionBase&>(step.getLegMotion(limb));
          // Express target position in world frame
          footPositionInWorld = adapter.transformPosition(endEffectorMotion.getFrameId(free_gait::ControlLevel::Position),
                  adapter.getWorldFrameId(), endEffectorMotion.getTargetPosition());
          isPlannedFor = true;
        }
      }
    }
    if (!isPlannedFor) {
      MELO_WARN_STREAM("Leg " << limb << " is neither grounded nor has a motion plan.");
      footPositionInWorld = adapter.getPositionWorldToFootInWorldFrame(limb);
    }
  }
  return footPositionInWorld;
}

} /* namespace locomotion_planner */
