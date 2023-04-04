/*
 * StairClimbing.hpp
 *
 *  Created on: Aug 30, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

#include "MotionGenerator.hpp"
#include "StairsGeometry.hpp"

// ROS
#include <ros/ros.h>

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/FreeGaitActionClient.hpp>
#include <free_gait_ros/AdapterRos.hpp>
#include <free_gait_ros/StepFrameConverter.hpp>
#include <free_gait_ros/StepRosConverter.hpp>

namespace free_gait_stair_climbing {

class StairClimbing
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  StairClimbing(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~StairClimbing();

 private:
  bool readStairsGeometryFromParameters(StairsGeometry& stairsGeometry);
  bool start();
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const free_gait_msgs::ExecuteStepsResult& result);

  ros::NodeHandle& nodeHandle_;
  //! Motion generation.
  std::unique_ptr<MotionGenerator> motionGenerator_;
  //! Geometry of the stairs.
  StairsGeometry stairsGeometry_;
  //! Action client.
  free_gait::FreeGaitActionClient actionClient_;
  //! Free Gait tools.
  free_gait::AdapterRos adapterRos_;
  free_gait::StepRosConverter rosConverter_;
  free_gait::StepFrameConverter frameConverter_;
};

} /* namespace */
