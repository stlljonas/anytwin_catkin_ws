/*
 * StairClimbing.cpp
 *
 *  Created on: Aug 30, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_stair_climbing/StairClimbing.hpp"

// ROS
#include <pluginlib/class_loader.h>

// STD
#include <functional>

using namespace free_gait;

namespace free_gait_stair_climbing {

StairClimbing::StairClimbing(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      actionClient_(nodeHandle),
      adapterRos_(nodeHandle),
      rosConverter_(adapterRos_.getAdapter()),
      frameConverter_(adapterRos_.getAdapter())
{
  // Register callback.
  actionClient_.registerCallback(nullptr, nullptr,
                                 std::bind(&StairClimbing::doneCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Read parameters.
  double baseHeight, footPlacementWidth;
  int nStepsBetweenFrontAndHindLegs;
  nodeHandle_.getParam("robot_parameters/base_height", baseHeight);
  nodeHandle_.getParam("robot_parameters/foot_placement_width", footPlacementWidth);
  nodeHandle_.getParam("robot_parameters/n_steps_between_front_and_hind_legs", nStepsBetweenFrontAndHindLegs);
  motionGenerator_.reset(new MotionGenerator(baseHeight, footPlacementWidth, nStepsBetweenFrontAndHindLegs));
  readStairsGeometryFromParameters(stairsGeometry_);

  // Generate motion.
  start();
}

StairClimbing::~StairClimbing() {}

bool StairClimbing::readStairsGeometryFromParameters(StairsGeometry& stairsGeometry)
{
  std::string frameId;
  nodeHandle_.getParam("stair_geometry/frame_id", frameId);
  stairsGeometry.setFrameId(frameId);

  int numberOfSteps;
  nodeHandle_.getParam("stair_geometry/number_of_steps", numberOfSteps);
  stairsGeometry.setNumberOfSteps(numberOfSteps);

  if (nodeHandle_.hasParam("stair_geometry/general_step")) {
    double rise, run;
    nodeHandle_.getParam("stair_geometry/general_step/rise", rise);
    nodeHandle_.getParam("stair_geometry/general_step/run", run);
    stairsGeometry.setGeneralStepGeometry(rise, run);
  }

  if (nodeHandle_.hasParam("stair_geometry/first_step")) {
    double rise, run;
    nodeHandle_.getParam("stair_geometry/first_step/rise", rise);
    nodeHandle_.getParam("stair_geometry/first_step/run", run);
    stairsGeometry.setFirstStepGeometry(rise, run);
  }

  if (nodeHandle_.hasParam("stair_geometry/last_step")) {
    double rise, run;
    nodeHandle_.getParam("stair_geometry/last_step/rise", rise);
    nodeHandle_.getParam("stair_geometry/last_step/run", run);
    stairsGeometry.setLastStepGeometry(rise, run);
  }

  return true;
}

bool StairClimbing::start()
{
  // Generate motion.
  ROS_INFO("Generate stair climbing motion.");
  motionGenerator_->computeMotion(stairsGeometry_);
  StepQueue queue = motionGenerator_->getMotion();

  // Transform from stairs to map frame.
  ROS_INFO("Transform from stairs to map frame.");
  if (!frameConverter_.adaptCoordinates(queue, stairsGeometry_.getFrameId(), "map")) return false;

  // Send action.
  ROS_INFO("Send action.");
  free_gait_msgs::ExecuteStepsGoal goal;
  rosConverter_.toMessage(queue, goal.steps);
  actionClient_.sendGoal(goal);

  return true;
}

void StairClimbing::doneCallback(const actionlib::SimpleClientGoalState& state,
                                 const free_gait_msgs::ExecuteStepsResult& result)
{
  ROS_INFO("Finished, shutting down.");
  nodeHandle_.shutdown();
}

} /* namespace */
