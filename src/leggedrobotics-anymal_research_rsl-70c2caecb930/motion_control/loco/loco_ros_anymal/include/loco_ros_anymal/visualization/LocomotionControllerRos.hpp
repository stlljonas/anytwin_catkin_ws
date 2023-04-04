/*
 * LocomotionControllerRos.hpp
 *
 *  Created on: Jan 26, 2015
 *      Author: C. Dario Bellicoso, Christian Gehring, Peter Fankhauser
 */

#pragma once

// ros
#include <ros/ros.h>

// loco
#include <loco/common/WholeBody.hpp>
#include <loco/common/TerrainModelBase.hpp>
#include "loco/common/FootprintGenerator.hpp"
#include <loco/locomotion_controller/LocomotionController.hpp>

// loco ros
#include "loco_ros/loco_ros.hpp"
#include <loco_ros/visualization/FrictionPyramid.hpp>
#include <loco_ros/visualization/FrictionCone.hpp>
#include <loco_ros/visualization/SupportPolygon.hpp>
#include <loco_ros/visualization/WholeBody.hpp>
#include <loco_ros/visualization/Torso.hpp>
#include <loco_ros/visualization/Feet.hpp>
#include <loco_ros/visualization/Transforms.hpp>
#include <loco_ros/visualization/TerrainModel.hpp>
#include <loco_ros/visualization/VirtualForces.hpp>
#include <loco_ros/visualization/SwingTrajectoryGeneratorSpline.hpp>
#include <loco_ros/visualization/ContactForces.hpp>

// loco_ros_anymal
#include <loco_ros_anymal/visualization/Ghost.hpp>
#include <loco_ros_anymal/visualization/WholeBodyDynamics.hpp>
#include <loco_ros_anymal/visualization/ContactSchedule.hpp>

#include <std_utils/timers/ChronoTimer.hpp>


namespace loco_ros_anymal {

class LocomotionControllerRos
{
 public:
  LocomotionControllerRos();
  virtual ~LocomotionControllerRos() = default;

  void setNodeHandle(const ros::NodeHandle& nodeHandle);
  virtual void initRos();

  // This function provides an interface which passes the modules to the second updateRos function.
  // If you add visualization, you need to adapt the second updateRos function.
  virtual void updateRos(const loco::LocomotionController& locomotionController);
  virtual void updateRos(const loco::WholeBody& wholeBody, const loco::TerrainModelBase& terrainModel,
                         const loco::HeadingGenerator& headingGenerator,
                         const loco::MotionControllerBase& motionController,
                         const loco::FootPlacementStrategyBase& footPlacementStrategy);

  virtual void publishRos();
  virtual void shutdownRos();

 protected:
  ros::NodeHandle nodeHandle_;

  loco_ros::SupportPolygon supportPolygonVisualizer_;
  loco_ros_anymal::Ghost ghostDesired_;
  loco_ros_anymal::Ghost ghostMeasured_;
  loco_ros::FrictionPyramid frictionPyramids_;
  loco_ros::FrictionCone frictionCones_;
  loco_ros::WholeBody wholeBodyVisualizer_;
  loco_ros::Torso torsoVisualizer_;
  loco_ros::Feet feetVisualizer_;
  loco_ros::Transforms transforms_;
  loco_ros::TerrainModel terrainModel_;
  loco_ros::VirtualForces virtualForces_;
  loco_ros::SwingTrajectoryGeneratorSpline swingTrajectorySplineVisualizer_;
  loco_ros_anymal::WholeBodyDynamics wholeBodyDynamicsVisualizer_;
  loco::FootprintGenerator footprintGenerator_;
  loco_ros::ContactForces contactForceVisualizer_;
//  loco_ros_anymal::ContactSchedule contactSchedule_;

  std_utils::HighResolutionClockTimer locoRosTimer_;
};

}  // namespace
