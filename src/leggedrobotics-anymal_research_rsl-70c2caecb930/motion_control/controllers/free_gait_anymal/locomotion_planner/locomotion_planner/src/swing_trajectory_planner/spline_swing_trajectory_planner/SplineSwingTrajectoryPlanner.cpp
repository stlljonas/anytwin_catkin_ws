/*
 * SplineSwingTrajectoryPlanner.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: Péter Fankhauser
 */

#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryPlanner.hpp"
#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryParameterization.hpp"
#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryObjectiveFunction.hpp"
#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryFunctionConstraints.hpp"
#include "locomotion_planner/common/geometry.hpp"
#include "locomotion_planner/common/type_defs.hpp"

#include <numopt_osqp/OperatorSplittingQuadraticProgramFunctionMinimizer.hpp>
#include <numopt_common/ConstrainedNonlinearProblem.hpp>
#include <numopt_sqp/SQPFunctionMinimizer.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

namespace locomotion_planner {

SplineSwingTrajectoryPlanner::SplineSwingTrajectoryPlanner(const free_gait::AdapterBase& adapter, Parameters& parameters,
                                                           std::shared_ptr<ElevationMapUser> elevationMapUser)
    : SwingTrajectoryPlannerBase(adapter, parameters),
      elevationMapUser_(elevationMapUser),
      stepCompleter_(stepParameters_, adapter)
{

}

SplineSwingTrajectoryPlanner::~SplineSwingTrajectoryPlanner()
{
}

bool SplineSwingTrajectoryPlanner::planSwingTrajectories(std::vector<free_gait::Step>& plan)
{
  if (!elevationMapUser_->isMapValid()) return false;
  bool success = true;
  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      free_gait::Step::LegMotions newLegMotions;
      for (auto& legMotion : step.getLegMotions()) {
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          free_gait::Footstep footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);  // Copy!

          if (!adapter_.isLegGrounded(legMotion.first)) {
            MELO_WARN("SplineSwingTrajectoryPlanner: Handling of this type of limb is not yet implemented!");
            continue;
          }

          footstep.updateStartPosition(adapter_.getPositionWorldToFootInWorldFrame(legMotion.first));
          stepCompleter_.setParameters(footstep);
          // Adapt height to start optimization from the highest allowed profile.
          footstep.setProfileHeight(parameters_.getSwingTrajectoryMaxHeight());
          footstep.compute(true);
          free_gait::EndEffectorTrajectory endEffectorTrajectory(legMotion.first);  // This will replace the footstep if planning successful.
          if (!planSwingTrajectory(footstep, endEffectorTrajectory)) {
            MELO_WARN("SplineSwingTrajectoryPlanner: Could not plan swing trajectory.");
            success = false;
            continue;
          }

          newLegMotions.insert(
              std::pair<LimbEnum, std::unique_ptr<free_gait::LegMotionBase>>(legMotion.first, std::move(endEffectorTrajectory.clone())));
        }
      }
      for (auto& legMotion : newLegMotions) {
        step.addLegMotion(*legMotion.second);
      }
    }
  }

  return success;
}

bool SplineSwingTrajectoryPlanner::planSwingTrajectory(const free_gait::Footstep& footstep,
                                                       free_gait::EndEffectorTrajectory& endEffectorTrajectory)
{
  MELO_DEBUG_STREAM("Planning swing trajectory for " << footstep.getLimb() << ".");

  // Optimize.
  std::shared_ptr<SplineSwingTrajectoryObjectiveFunction> objective(
      new SplineSwingTrajectoryObjectiveFunction(
          elevationMapUser_->getMap(), elevationMapUser_->getSignedDistanceField(parameters_.getCollisionLayer()),
          parameters_));
  objective->setTrajectoryParameters(footstep.getStartPosition(), footstep.getTargetPosition(),
                                     footstep.getStartVelocity(), footstep.getTargetVelocity(),
                                     footstep.getAverageVelocity(), footstep.getMinimumDuration());
  std::shared_ptr<SplineSwingTrajectoryFunctionConstraints> constraints(new SplineSwingTrajectoryFunctionConstraints(1, parameters_));
  constraints->setTrajectoryParameters(footstep.getStartPosition(), footstep.getTargetPosition());
  numopt_common::ConstrainedNonlinearProblem problem(objective, constraints);
  std::shared_ptr<numopt_osqp::OperatorSplittingQuadraticProgramFunctionMinimizer> qpSolver(new numopt_osqp::OperatorSplittingQuadraticProgramFunctionMinimizer);
  numopt_sqp::SQPFunctionMinimizer solver(qpSolver, 2000, 0.1, 20);
  SplineSwingTrajectoryParameterization params(1);
  std::vector<free_gait::Footstep::ValueType> knotPositions;
  knotPositions.insert(knotPositions.begin(), ++footstep.getKnotValues().begin(), --footstep.getKnotValues().end());
  params.setKnotPositions(knotPositions);
  objective->setInitialSolution(knotPositions);
  constraints->setInitialSolution(knotPositions);
  double functionValue;
  if (!solver.minimize(&problem, params, functionValue)) {
    return false;
  }

  // Setup end effector trajectory motion command.
  SplineSwingTrajectoryParameterization splineParameterization(params); // Need a copy.
  std::unordered_map<free_gait::ControlLevel, std::string, free_gait::EnumClassHash> frameIds;
  frameIds[free_gait::ControlLevel::Position] = adapter_.getWorldFrameId();
  frameIds[free_gait::ControlLevel::Velocity] = adapter_.getWorldFrameId();
  std::vector<free_gait::EndEffectorTrajectory::Time> times;
  std::unordered_map<free_gait::ControlLevel, std::vector<free_gait::EndEffectorTrajectory::ValueType>, free_gait::EnumClassHash> values;
  std::vector<curves::CubicHermiteE3Curve::ValueType> positions;
  splineParameterization.getTimesAndValues(footstep.getStartPosition(), footstep.getTargetPosition(),
                                           footstep.getAverageVelocity(), footstep.getMinimumDuration(),
                                           times, positions);
  values[free_gait::ControlLevel::Position] = positions;
  values[free_gait::ControlLevel::Velocity].resize(positions.size(), LinearVelocity().vector());
  values[free_gait::ControlLevel::Velocity].front() = footstep.getStartVelocity().vector();
  values[free_gait::ControlLevel::Velocity].back() = footstep.getTargetVelocity().vector();
  endEffectorTrajectory.setTrajectory(frameIds, times, values);
  return true;
}

}
