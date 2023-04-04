/*
 * MotionPlanVisualizer.hpp
 *
 *  Created on: Feb 7, 2017
 *      Author: dbellicoso
 */

#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

// loco
#include <loco/common/typedefs.hpp>
#include "zmp_optimizer/zmp_optimizer.hpp"
#include "zmp_optimizer/MotionPlan.hpp"
#include "zmp_optimizer/SupportPolygon.hpp"

// loco ros
#include <loco_ros/loco_ros.hpp>
#include <loco_ros/visualization/ModuleRos.hpp>

// curves
#include "curves/PolynomialSplineContainer.hpp"

// robot utils
#include <robot_utils/geometry/geometry.hpp>

// stl
#include <string>


namespace anymal_ctrl_dynamic_gaits_ros {

class MotionPlanVisualizer : public ModuleRos {
 public:
  MotionPlanVisualizer();
  virtual ~MotionPlanVisualizer() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic = "/dynamic_gaits_ros/");

  bool update(const zmp::MotionPlan& motionPlan);
  bool publish();

  void setColorVector(std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors);
  void setColorVectorId(unsigned int colorId);

  void setVisualizeOnGround(bool visualizeOnGround);

 protected:

  bool initPublishers(const std::string& topic);
  bool initMsgs();

  unsigned int computeActivePolygonId(double splineTime, const std::vector<double>& polygonTimes) const;

  ros::NodeHandle nodeHandle_;

  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedComPosition_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedComPositionOnPlane_;
  std::pair<ros::Publisher, visualization_msgs::Marker>      plannedComPositionLineStrip_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedComVelocity_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedComAcceleration_;
  std::pair<ros::Publisher, geometry_msgs::PoseArray> plannedComOrientation_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedComEulerVel_;
  std::pair<ros::Publisher, geometry_msgs::PoseArray> pathRegularizerOrientation_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedZmpPosition_;
  std::pair<ros::Publisher, visualization_msgs::Marker>      plannedZmpPositionLineStrip_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> pathRegularizer_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> pathRegularizerOnPlane_;

  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedInitialState_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedFinalState_;

  unsigned int ratio_;
  unsigned int numMotionPlanPositionSamples_;
  unsigned int numMotionPlanArrowSamples_;

  std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>> colors_;
  loco::Vector colorsFlightPhase_;

  bool visualizeOnGround_;
  unsigned int colorId_;
};

} /* namespace anymal_ctrl_dynamic_gaits_ros */
