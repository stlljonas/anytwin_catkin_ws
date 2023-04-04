/*
 * LocomotionControllerElevationMapRos.hpp
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

// loo ros elevation map
#include <loco_elevation_map_ros/FootholdSearch.hpp>

// loco_ros_anymal
#include <loco_ros_anymal/visualization/Ghost.hpp>
#include <loco_ros_anymal/visualization/WholeBodyDynamics.hpp>
#include <loco_ros_anymal/visualization/LocomotionControllerRos.hpp>


namespace loco_elevation_map_ros {

class LocomotionControllerElevationMapRos : public loco_ros_anymal::LocomotionControllerRos {
 private:
  using Base = loco_ros_anymal::LocomotionControllerRos;

 public:
  LocomotionControllerElevationMapRos();
  ~LocomotionControllerElevationMapRos() override = default;
  void initRos() override;

  using Base::updateRos;
  void updateRos(const loco::WholeBody& wholeBody, const loco::TerrainModelBase& terrainModel,
                 const loco::HeadingGenerator& headingGenerator,
                 const loco::MotionControllerBase& motionController,
                 const loco::FootPlacementStrategyBase& footPlacementStrategy) override;
  void publishRos() override;
  void shutdownRos() override;

 protected:
  loco_ros::FootholdSearch footholdSearchVisualizer_;
};

}  // namespace
