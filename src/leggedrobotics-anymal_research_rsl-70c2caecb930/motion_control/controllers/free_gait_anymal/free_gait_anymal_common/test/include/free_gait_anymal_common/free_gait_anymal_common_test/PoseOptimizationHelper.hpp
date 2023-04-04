/*
 * PoseOptimizationHelper.hpp
 *
 *  Created on: Jul 7, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/free_gait_ros.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_ros/PolygonRosConverter.hpp>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace free_gait {

class PoseOptimizationHelper
{
 public:
  using OptimizationStepCallbackFunction = typename PoseOptimizationSQP::OptimizationStepCallbackFunction;

  PoseOptimizationHelper()
      : nodeHandle_("~"),
        adapterRos_(nodeHandle_, AdapterRos::AdapterType::Preview),
        initialStateRosPublisher_(nodeHandle_, *adapterRos_.getAdapterPtr()),
        transientStateRosPublisher_(nodeHandle_, *adapterRos_.getAdapterPtr()),
        finalStateRosPublisher_(nodeHandle_, *adapterRos_.getAdapterPtr()),
        hidePreviousResult_(true),
        sleepDurationForOptimizationStep_(0.5)
  {
    initialStateRosPublisher_.setTfPrefix("initial");
    transientStateRosPublisher_.setTfPrefix("transient");
    finalStateRosPublisher_.setTfPrefix("final");
    stancePublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/stance", 1);
    centerOfMassPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/center_of_mass", 1);
    AdapterBase& adapter(*adapterRos_.getAdapterPtr());
    optimizationGeometric_.reset(new PoseOptimizationGeometric(adapter));
    optimizationQP_.reset(new PoseOptimizationQP(adapter));
    constraintsChecker_.reset(new PoseConstraintsChecker(adapter));
    optimizationSQP_.reset(new PoseOptimizationSQP(adapter));
    State state;
    state.initialize(adapter.getLimbs(), adapter.getBranches());
    state.setZero();
    setDefaultJointPosition(state);
    optimizationQP_->setCurrentState(state);
    optimizationSQP_->setCurrentState(state);
    constraintsChecker_->setCurrentState(state);
    optimizationStepCallback_ = std::bind(&PoseOptimizationHelper::optimizationStepCallback, this,
                                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                          std::placeholders::_4);
    optimizationSQP_->registerOptimizationStepCallback(optimizationStepCallback_);
    nominalStanceInBaseFrame_ = Stance({
      {LimbEnum::LF_LEG, Position(0.33, 0.22, -0.48)},
      {LimbEnum::RF_LEG, Position(0.33, -0.22, -0.48)},
      {LimbEnum::LH_LEG, Position(-0.33, 0.22, -0.48)},
      {LimbEnum::RH_LEG, Position(-0.33, -0.22, -0.48)} });

    optimizationGeometric_->setNominalStance(nominalStanceInBaseFrame_);
    optimizationQP_->setNominalStance(nominalStanceInBaseFrame_);
    optimizationSQP_->setNominalStance(nominalStanceInBaseFrame_);
    PoseConstraintsChecker::LimbLengths minLimbLenghts, maxLimbLenghts;
    for (const auto& limb : adapter.getLimbs()) {
      minLimbLenghts[limb] = 0.2;
      maxLimbLenghts[limb] = 0.59;
    }
    constraintsChecker_->setLimbLengthConstraints(minLimbLenghts, maxLimbLenghts);
    optimizationSQP_->setLimbLengthConstraints(minLimbLenghts, maxLimbLenghts);
  }

  virtual ~PoseOptimizationHelper() = default;

  void setSleepDurationForOptimizationStep(const ros::Duration& duration)
  {
    sleepDurationForOptimizationStep_ = duration;
  }

  void setNominalStanceInBaseFrame(const Stance& nominalStanceInBaseFrame)
  {
    nominalStanceInBaseFrame_ = nominalStanceInBaseFrame;
    optimizationGeometric_->setNominalStance(nominalStanceInBaseFrame);
    optimizationQP_->setNominalStance(nominalStanceInBaseFrame);
    optimizationSQP_->setNominalStance(nominalStanceInBaseFrame);
  }

  void setLimbLengthConstraints(const PoseConstraintsChecker::LimbLengths& minLimbLengths,
                                const PoseConstraintsChecker::LimbLengths& maxLimbLengths) {
    constraintsChecker_->setLimbLengthConstraints(minLimbLengths, maxLimbLengths);
    optimizationSQP_->setLimbLengthConstraints(minLimbLengths, maxLimbLengths);
  }

  void setFootholdsToReach(const Stance& footholdsToReach)
  {
    optimizationGeometric_->setStance(footholdsToReach);
    optimizationGeometric_->setStanceForOrientation(footholdsToReach);
    optimizationQP_->setStance(footholdsToReach);
    constraintsChecker_->setStance(footholdsToReach);
    optimizationSQP_->setStance(footholdsToReach);
  }

  void setFootholdsInSupport(const Stance& footholdsInSupport, double supportMargin = 0.0)
  {
    // Set support region.
    grid_map::Polygon supportPolygon;
    supportPolygon.setFrameId(adapterRos_.getAdapterPtr()->getWorldFrameId());
    supportPolygon.setTimestamp(ros::Time::now().toNSec());
    std::vector<Position> footholdsOrdered;
    getFootholdsCounterClockwiseOrdered(footholdsInSupport, footholdsOrdered);
    for (auto foothold : footholdsOrdered) {
      supportPolygon.addVertex(foothold.vector().head<2>());
    }
    bool isLinePolygon = false;
    if (supportPolygon.nVertices() == 2) {
      supportPolygon.thickenLine(0.002);
      isLinePolygon = true;
    }
    grid_map::Polygon supportRegion = supportPolygon;
    if (!isLinePolygon) supportRegion.offsetInward(supportMargin);
    optimizationGeometric_->setSupportRegion(supportRegion);
    optimizationQP_->setSupportRegion(supportRegion);
    constraintsChecker_->setSupportRegion(supportRegion);
    optimizationSQP_->setSupportRegion(supportRegion);

    // Visualize stance.
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;
    color.g = 1.0;
    color.a = 0.7;
    grid_map::PolygonRosConverter::toTriangleListMarker(supportPolygon, color, 0.0, marker);
    stancePublisher_.publish(marker);
  }

  bool optimize(Pose& pose)
  {
    if (hidePreviousResult_) {
      AdapterBase& adapter(*adapterRos_.getAdapterPtr());
      State state;
      state.initialize(adapter.getLimbs(), adapter.getBranches());
      state.setZero();
      Position position;
      position.z() = -1e6;
      state.setPositionWorldToBaseInWorldFrame(position);
      finalStateRosPublisher_.publish(state);
    }

    optimizationGeometric_->optimize(pose);
    const bool success1 = optimizationQP_->optimize(pose);
    if (constraintsChecker_->check(pose)) {
      optimizationSQP_->callExternalOptimizationStepCallbackWithPose(pose, 0, 0.0, true);
      return true;
    }
    const bool success2 = optimizationSQP_->optimize(pose);
    std::cout << "Number of iterations: " << optimizationSQP_->getNumberOfIterations() << ", Optimization Duration: " << optimizationSQP_->getOptimizationDuration() / 1000.0 << " ms." << std::endl;
    std::cout << "===================" << std::endl << std::endl;
//    std::cout << optimizationSQP_->getNumberOfIterations() << ", " << optimizationSQP_->getOptimizationDuration() / 1000.0 << " \n";
    if (!success2) {
      std::cerr << "Pose optimization failed.\n";
    }
    return success1 && success2;
  }

  void setDefaultJointPosition(State& state)
  {
    state.setJointPositionsForLimb(LimbEnum::LF_LEG, JointPositionsLeg(0.0, 0.7, -1.5));
    state.setJointPositionsForLimb(LimbEnum::RF_LEG, JointPositionsLeg(0.0, 0.7, -1.5));
    state.setJointPositionsForLimb(LimbEnum::LH_LEG, JointPositionsLeg(0.0, -0.7, 1.5));
    state.setJointPositionsForLimb(LimbEnum::RH_LEG, JointPositionsLeg(0.0, -0.7, 1.5));
  }

  AdapterBase* getAdapterRosPtr() {
    return adapterRos_.getAdapterPtr();
  }

 protected:

  void optimizationStepCallback(const size_t iterationStep,
                                const State& state,
                                const double functionValue,
                                const bool finalIteration)
  {
    // Publish CoM.
    const Position comPosition = adapterRos_.getAdapterPtr()->getCenterOfMassInWorldFrame();
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.a = 1.0;
    visualization_msgs::MarkerArray comMarker = RosVisualization::getComWithProjectionMarker(
        comPosition, adapterRos_.getAdapterPtr()->getWorldFrameId(), color, 0.05, 0.7, 0.005);
    centerOfMassPublisher_.publish(comMarker);

    // Visualize robot.
    transientStateRosPublisher_.publish(state);
    if (iterationStep == 0) initialStateRosPublisher_.publish(state);
    if (finalIteration) finalStateRosPublisher_.publish(state);
    sleepDurationForOptimizationStep_.sleep();
  }

  ros::NodeHandle nodeHandle_;
  AdapterRos adapterRos_;
  StateRosPublisher initialStateRosPublisher_;
  StateRosPublisher transientStateRosPublisher_;
  StateRosPublisher finalStateRosPublisher_;
  ros::Publisher stancePublisher_;
  ros::Publisher centerOfMassPublisher_;
  std::unique_ptr<PoseOptimizationGeometric> optimizationGeometric_;
  std::unique_ptr<PoseOptimizationQP> optimizationQP_;
  std::unique_ptr<PoseConstraintsChecker> constraintsChecker_;
  std::unique_ptr<PoseOptimizationSQP> optimizationSQP_;
  OptimizationStepCallbackFunction optimizationStepCallback_;
  Stance nominalStanceInBaseFrame_;
  bool hidePreviousResult_;
  ros::Duration sleepDurationForOptimizationStep_;
};

}
