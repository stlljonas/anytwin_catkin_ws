/*
 * RosControllerInterfaceBase.cpp
 *
 *  Created on: Jan 26, 2015
 *      Author: C. Dario Bellicoso, Peter Fankhauser
 */

// loco
#include "loco_elevation_map_ros/LocomotionControllerElevationMapRos.hpp"


namespace loco_elevation_map_ros {

LocomotionControllerElevationMapRos::LocomotionControllerElevationMapRos()
    : Base()
{

}

void LocomotionControllerElevationMapRos::shutdownRos() {
  Base::shutdownRos();
  footholdSearchVisualizer_.shutdown();
}


void LocomotionControllerElevationMapRos::initRos() {
  Base::initRos();

  bool isSimulation = false;
  nodeHandle_.param<bool>("controller/simulation", isSimulation, false);

  footholdSearchVisualizer_.initialize(nodeHandle_, isSimulation);
}


void LocomotionControllerElevationMapRos::updateRos(
    const loco::WholeBody& wholeBody,
    const loco::TerrainModelBase& terrainModel,
    const loco::HeadingGenerator& headingGenerator,
    const loco::MotionControllerBase& motionController,
    const loco::FootPlacementStrategyBase& footPlacementStrategy) {
  Base::updateRos(
      wholeBody, terrainModel, headingGenerator, motionController, footPlacementStrategy);

  const loco::Legs& legs = wholeBody.getLegs();

  try {
    footholdSearchVisualizer_.update(legs, dynamic_cast<const loco::TerrainModelElevationMap&>(terrainModel));
  } catch (...) {

  }

}

void LocomotionControllerElevationMapRos::publishRos() {
  Base::publishRos();
  footholdSearchVisualizer_.publish();
}


} /* namespace loco_ros_anymal */
