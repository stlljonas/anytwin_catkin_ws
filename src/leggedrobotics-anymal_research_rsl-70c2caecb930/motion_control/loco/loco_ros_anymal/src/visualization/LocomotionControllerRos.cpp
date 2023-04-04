/*
 * LocomotionControllerRos.cpp
 *
 *  Created on: Jan 26, 2015
 *      Author: C. Dario Bellicoso, Peter Fankhauser
 */

// loco
#include "loco_ros_anymal/visualization/LocomotionControllerRos.hpp"


namespace loco_ros_anymal {

LocomotionControllerRos::LocomotionControllerRos()
    : ghostDesired_(ghost::StateEnum::Desired),
      ghostMeasured_(ghost::StateEnum::Measured),
      frictionPyramids_(0.8, 0.12),
      frictionCones_(0.8, 0.12)
{
  locoRosTimer_.setAlpha(0.8);
}

void LocomotionControllerRos::shutdownRos()
{
  frictionPyramids_.shutdown();
  frictionCones_.shutdown();
  supportPolygonVisualizer_.shutdown();
  ghostDesired_.shutdown();
  ghostMeasured_.shutdown();
  wholeBodyVisualizer_.shutdown();
  torsoVisualizer_.shutdown();
  feetVisualizer_.shutdown();
  transforms_.shutdown();
  terrainModel_.shutdown();
  virtualForces_.shutdown();
  swingTrajectorySplineVisualizer_.shutdown();
  wholeBodyDynamicsVisualizer_.shutdown();
  contactForceVisualizer_.shutdown();
//  contactSchedule_.shutdown();
}


void LocomotionControllerRos::setNodeHandle(const ros::NodeHandle& nodeHandle)
{
  nodeHandle_ = nodeHandle;
}

void LocomotionControllerRos::initRos() {
  bool isSimulation = false;
  nodeHandle_.param<bool>("controller/simulation", isSimulation, false);

  frictionPyramids_.initialize(nodeHandle_, "/loco_ros/friction_pyramids");
  frictionCones_.initialize(nodeHandle_, "/loco_ros/friction_cones");
  supportPolygonVisualizer_.initialize(nodeHandle_, "/loco_ros/support_polygon");
  ghostDesired_.initialize(nodeHandle_, "/ghost_desired/anymal_state");
  ghostMeasured_.initialize(nodeHandle_, "/ghost_measured/anymal_state");
  wholeBodyVisualizer_.initialize(nodeHandle_);
  torsoVisualizer_.initialize(nodeHandle_, "/loco_ros/torso");
  feetVisualizer_.initialize(nodeHandle_, isSimulation);
  transforms_.initialize(nodeHandle_);
  terrainModel_.initialize(nodeHandle_);
  virtualForces_.initialize(nodeHandle_);
  swingTrajectorySplineVisualizer_.initialize(nodeHandle_, "/loco_ros/swing_traj");
  wholeBodyDynamicsVisualizer_.initialize(nodeHandle_);
  contactForceVisualizer_.initialize(nodeHandle_, {"lf", "rf", "lh", "rh"});
//  contactSchedule_.initialize(nodeHandle_);
}

void LocomotionControllerRos::updateRos(const loco::LocomotionController& locomotionController) {
  updateRos(locomotionController.getWholeBody(),
            locomotionController.getTerrainModel(),
            locomotionController.getHeadingGenerator(),
            locomotionController.getMotionController(),
            locomotionController.getFootPlacementStrategy());
}

void LocomotionControllerRos::updateRos(
    const loco::WholeBody& wholeBody,
    const loco::TerrainModelBase& terrainModel,
    const loco::HeadingGenerator& headingGenerator,
    const loco::MotionControllerBase& motionController,
    const loco::FootPlacementStrategyBase& footPlacementStrategy) {

  // save current time to stamp the outgoing messages
  const ros::Time timestamp = ros::Time::now();

  auto& torso = wholeBody.getTorso();
  auto& legs = wholeBody.getLegs();

  // update and visualize friction pyramids and cones
//  locoRosTimer_.pinTime("footprint");
  footprintGenerator_.update(legs, headingGenerator);
//  locoRosTimer_.splitTime("footprint");
  const loco::Vector& footprintHeadingDirectionInWorldFrame = footprintGenerator_.getFootprintHeadingDirectionInWorldFrame();

//  std::cout << "fpyr subs: " << frictionPyramids_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("f_pyramids");
  frictionPyramids_.update(footprintHeadingDirectionInWorldFrame, legs, terrainModel);
//  locoRosTimer_.splitTime("f_pyramids");

//  std::cout << "fcones subs: " << frictionCones_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("f_cones");
  frictionCones_.update(footprintHeadingDirectionInWorldFrame, legs, terrainModel);
//  locoRosTimer_.splitTime("f_cones");

//  std::cout << "spoly subs: " << supportPolygonVisualizer_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("support_polygon");
  supportPolygonVisualizer_.update(torso, legs, false);
//  locoRosTimer_.splitTime("support_polygon");

//  std::cout << "ghost d subs: " << ghostDesired_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("ghost_des");
  ghostDesired_.update(wholeBody, timestamp);
//  locoRosTimer_.splitTime("ghost_des");

//  std::cout << "ghost m subs: " << ghostMeasured_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("ghost_meas");
  ghostMeasured_.update(wholeBody, timestamp);
//  locoRosTimer_.splitTime("ghost_meas");

//  std::cout << "wb subs: " << wholeBodyVisualizer_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("whole_body");
  wholeBodyVisualizer_.update(wholeBody);
//  locoRosTimer_.splitTime("whole_body");

//  std::cout << "torso subs: " << torsoVisualizer_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("torso");
  torsoVisualizer_.update(torso, legs, terrainModel);
//  locoRosTimer_.splitTime("torso");

//  std::cout << "feet subs: " << feetVisualizer_.getNumSubscribers() << std::endl;
//  locoRosTimer_.pinTime("feet");
  feetVisualizer_.update(legs);
//  locoRosTimer_.splitTime("feet");

//  locoRosTimer_.pinTime("tfs");
  transforms_.update(wholeBody, terrainModel, headingGenerator);
//  locoRosTimer_.splitTime("tfs");

//  locoRosTimer_.pinTime("terrain");
  terrainModel_.update(wholeBody, terrainModel);
//  locoRosTimer_.splitTime("terrain");

  try {
//    locoRosTimer_.pinTime("forces");
    virtualForces_.update(dynamic_cast<const loco::VirtualModelController&>(motionController), torso);
//    locoRosTimer_.splitTime("forces");
  } catch (...) {
    ;
  }

  try {
//    locoRosTimer_.pinTime("swing_traj");
    swingTrajectorySplineVisualizer_.update(
        dynamic_cast<const loco::SwingTrajectoryGeneratorSpline&>(footPlacementStrategy.getSwingTrajectoryGenerator()),
        torso);
//    locoRosTimer_.splitTime("swing_traj");
  } catch (...) {
    ;
  }

//  locoRosTimer_.pinTime("wb_dyn");
  wholeBodyDynamicsVisualizer_.update(wholeBody);
//  locoRosTimer_.splitTime("wb_dyn");

//  locoRosTimer_.pinTime("forces");
  contactForceVisualizer_.update(wholeBody.getLimbs());
//  locoRosTimer_.splitTime("forces");



//  std::cout << locoRosTimer_ << std::endl;

//  contactSchedule_.update(locomotionController.getGaitPattern(), legs);
}

void LocomotionControllerRos::publishRos()
{
  frictionPyramids_.publish();
  frictionCones_.publish();
  supportPolygonVisualizer_.publish();
  ghostDesired_.publish();
  ghostMeasured_.publish();
  wholeBodyVisualizer_.publish();
  torsoVisualizer_.publish();
  feetVisualizer_.publish();
  transforms_.publish();
  terrainModel_.publish();
  virtualForces_.publish();
  swingTrajectorySplineVisualizer_.visualize();
  wholeBodyDynamicsVisualizer_.publish();
  contactForceVisualizer_.publish();
//  contactSchedule_.publish();
}


} /* namespace loco_ros_anymal */
