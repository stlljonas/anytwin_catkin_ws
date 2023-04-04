/*
 * MotionPlanVisualizer.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: dbellicoso
 */

// anymal ctrl dynamic gaits ros
#include "motion_generation_ros/MotionPlanVisualizer.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace anymal_ctrl_dynamic_gaits_ros {

MotionPlanVisualizer::MotionPlanVisualizer()
  :  nodeHandle_(),
     ratio_(5u),
     numMotionPlanPositionSamples_(90u),
     numMotionPlanArrowSamples_(numMotionPlanPositionSamples_/ratio_),
     colorsFlightPhase_(0.3, 0.3, 0.3),
     visualizeOnGround_(false),
     colorId_(0u)
{
  publisherRefs_.push_back(plannedComPosition_.first);
  publisherRefs_.push_back(plannedComPositionOnPlane_.first);
  publisherRefs_.push_back(plannedComPositionLineStrip_.first);
  publisherRefs_.push_back(plannedComVelocity_.first);
  publisherRefs_.push_back(plannedComAcceleration_.first);
  publisherRefs_.push_back(plannedComOrientation_.first);
  publisherRefs_.push_back(plannedComEulerVel_.first);
  publisherRefs_.push_back(pathRegularizerOrientation_.first);
  publisherRefs_.push_back(plannedZmpPosition_.first);
  publisherRefs_.push_back(plannedZmpPositionLineStrip_.first);
  publisherRefs_.push_back(pathRegularizer_.first);
  publisherRefs_.push_back(pathRegularizerOnPlane_.first);
  publisherRefs_.push_back(plannedInitialState_.first);
  publisherRefs_.push_back(plannedFinalState_.first);
}

void MotionPlanVisualizer::setColorVector(std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors) {
  colors_ = colors;
}

void MotionPlanVisualizer::setColorVectorId(unsigned int colorId) {
  colorId_ = colorId;
}


bool MotionPlanVisualizer::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  // Set nodehandle.
  nodeHandle_ = nodeHandle;

  initPublishers(topic);
  initMsgs();

  return true;
}

bool MotionPlanVisualizer::initPublishers(const std::string& topic) {
  // Initialize publishers.
  plannedComPosition_.first          = nodeHandle_.advertise<decltype(plannedComPosition_.second)>(topic + "planned_com_pos_markers", 1, true);
  plannedComPositionOnPlane_.first   = nodeHandle_.advertise<decltype(plannedComPositionOnPlane_.second)>(topic + "planned_com_pos_on_plane_markers", 1, true);
  plannedComPositionLineStrip_.first = nodeHandle_.advertise<decltype(plannedComPositionLineStrip_.second)>(topic + "planned_com_pos_line_strip_markers", 1, true);
  plannedComVelocity_.first          = nodeHandle_.advertise<decltype(plannedComVelocity_.second)>(topic + "planned_com_vel_markers", 1, true);
  plannedComAcceleration_.first      = nodeHandle_.advertise<decltype(plannedComAcceleration_.second)>(topic + "planned_com_acc_markers", 1, true);
  plannedComOrientation_.first       = nodeHandle_.advertise<decltype(plannedComOrientation_.second)>(topic + "planned_com_orientation_markers", 1, true);
  plannedComEulerVel_.first          = nodeHandle_.advertise<decltype(plannedComEulerVel_.second)>(topic + "planned_com_euler_vel_markers", 1, true);
  pathRegularizerOrientation_.first  = nodeHandle_.advertise<decltype(pathRegularizerOrientation_.second)>(topic + "path_regularizer_orientation_markers", 1, true);
  plannedZmpPosition_.first          = nodeHandle_.advertise<decltype(plannedZmpPosition_.second)>(topic + "planned_zmp_pos_markers", 1, true);
  plannedZmpPositionLineStrip_.first = nodeHandle_.advertise<decltype(plannedZmpPositionLineStrip_.second)>(topic + "planned_zmp_pos_line_strip_markers", 1, true);
  pathRegularizer_.first             = nodeHandle_.advertise<decltype(pathRegularizer_.second)>(topic + "path_regularizer", 1, true);
  pathRegularizerOnPlane_.first      = nodeHandle_.advertise<decltype(pathRegularizerOnPlane_.second)>(topic + "path_regularizer_on_plane", 1, true);

  plannedInitialState_.first         = nodeHandle_.advertise<decltype(plannedInitialState_.second)>(topic + "planned_initial_state", 1, true);
  plannedFinalState_.first           = nodeHandle_.advertise<decltype(plannedFinalState_.second)>(topic + "planned_final_state", 1, true);

  return true;
}

bool MotionPlanVisualizer::initMsgs() {
  // Initialize markers.
  plannedComPosition_.second.markers.clear();
  plannedComPositionOnPlane_.second.markers.clear();
  plannedComPositionLineStrip_.second.points.clear();
  plannedComVelocity_.second.markers.clear();
  plannedComAcceleration_.second.markers.clear();
  plannedComOrientation_.second.poses.clear();
  plannedComEulerVel_.second.markers.clear();
  pathRegularizerOrientation_.second.poses.clear();
  plannedZmpPosition_.second.markers.clear();
  plannedZmpPositionLineStrip_.second.points.clear();
  pathRegularizer_.second.markers.clear();
  pathRegularizerOnPlane_.second.markers.clear();

  plannedInitialState_.second.markers.clear();
  plannedFinalState_.second.markers.clear();

  plannedComPositionLineStrip_.second = loco_ros::getInitializedMarker(visualization_msgs::Marker::LINE_STRIP, "odom", "planned_com_line_strips", 0);
  plannedComPositionLineStrip_.second.scale.x = 0.0008;
  plannedComPositionLineStrip_.second.color.r = 0.0;
  plannedComPositionLineStrip_.second.color.g = 0.0;
  plannedComPositionLineStrip_.second.color.b = 0.0;

  plannedZmpPositionLineStrip_.second = loco_ros::getInitializedMarker(visualization_msgs::Marker::LINE_STRIP, "odom", "planned_zmp_line_strips", 0);
  plannedZmpPositionLineStrip_.second.scale.x = 0.0008;
  plannedZmpPositionLineStrip_.second.color.r = 0.0;
  plannedZmpPositionLineStrip_.second.color.g = 0.0;
  plannedZmpPositionLineStrip_.second.color.b = 0.0;

  for (int k=0; k<numMotionPlanPositionSamples_; ++k) {
    plannedComPosition_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "planned_com_motion_position", k, 0.002));
    plannedComPositionOnPlane_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "planned_com_motion_position_on_plane", k, 0.002));
    plannedZmpPosition_.second.markers.push_back(loco_ros::getInitializedReferenceSphereMarker("odom", "planned_zmp_motion_position", k, 0.002));
    pathRegularizer_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "path_regularizer", k, 0.002));
    pathRegularizerOnPlane_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "path_regularizer_on_plane", k, 0.002));
  }

  plannedComOrientation_.second.header.frame_id = "odom";
  pathRegularizerOrientation_.second.header.frame_id = "odom";
  for (int k=0; k<numMotionPlanArrowSamples_; ++k) {
    plannedComVelocity_.second.markers.push_back(loco_ros::getInitializedReferenceArrowMarker("odom", "planned_com_motion_velocity", k, 0.0015, 0.0, 1.0, 0.0));
    plannedComAcceleration_.second.markers.push_back(loco_ros::getInitializedReferenceArrowMarker("odom", "planned_com_motion_acceleration", k, 0.0015, 0.0, 0.0, 1.0));
    plannedComOrientation_.second.poses.push_back(geometry_msgs::Pose());
    plannedComEulerVel_.second.markers.push_back(loco_ros::getInitializedReferenceArrowMarker("odom", "planned_com_motion_euler_vel", k, 0.0015, 0.0, 0.0, 1.0));
    pathRegularizerOrientation_.second.poses.push_back(geometry_msgs::Pose());
  }

  for (auto& marker : plannedComVelocity_.second.markers) {
    marker.points.push_back(geometry_msgs::Point());
    marker.points.push_back(geometry_msgs::Point());
  }

  for (auto& marker : plannedComAcceleration_.second.markers) {
    marker.points.push_back(geometry_msgs::Point());
    marker.points.push_back(geometry_msgs::Point());
  }

  for (auto& marker : plannedComEulerVel_.second.markers) {
    marker.points.push_back(geometry_msgs::Point());
    marker.points.push_back(geometry_msgs::Point());
  }

  plannedInitialState_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "initial_state_pos", 0, 0.0015));
  plannedInitialState_.second.markers.push_back(loco_ros::getInitializedReferenceArrowMarker("odom", "initial_state_vel", 0, 0.0015));
  plannedInitialState_.second.markers.back().points.push_back(geometry_msgs::Point());
  plannedInitialState_.second.markers.back().points.push_back(geometry_msgs::Point());
  plannedInitialState_.second.markers.push_back(loco_ros::getInitializedReferenceArrowMarker("odom", "initial_state_acc", 0, 0.0015));
  plannedInitialState_.second.markers.back().points.push_back(geometry_msgs::Point());
  plannedInitialState_.second.markers.back().points.push_back(geometry_msgs::Point());

  plannedFinalState_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "final_state_pos", 0, 0.0015));
  plannedFinalState_.second.markers.push_back(loco_ros::getInitializedReferenceArrowMarker("odom", "final_state_vel", 0, 0.0015));
  plannedFinalState_.second.markers.back().points.push_back(geometry_msgs::Point());
  plannedFinalState_.second.markers.back().points.push_back(geometry_msgs::Point());
  plannedFinalState_.second.markers.push_back(loco_ros::getInitializedReferenceArrowMarker("odom", "final_state_acc", 0, 0.0015));
  plannedFinalState_.second.markers.back().points.push_back(geometry_msgs::Point());
  plannedFinalState_.second.markers.back().points.push_back(geometry_msgs::Point());

  return true;
}

bool MotionPlanVisualizer::update(const zmp::MotionPlan& motionPlan) {
  if (isNotSubscribed()) {
    return true;
  }

  if (!motionPlan.didOptimizationSucceeded()) {
    return true;
  }

  // Check if splines are empty
  bool isCogPlan         = !motionPlan.getComStateInPlaneFrame().isEmpty();
  bool isPathRegularizer = !motionPlan.getPathRegularizerInPlaneFrame().isEmpty();

  // Scaling
  constexpr double scaleVelocity = 0.3;
  constexpr double scaleAcceleration = 0.1;
  constexpr double scaleOrientation = 1.5;

  // Extract sampling information
  const double splineDuration = motionPlan.getComStateInPlaneFrame().getContainerDuration();
  const double startTime = motionPlan.getComStateInPlaneFrame().getContainerTime();
  const double splineDt = (splineDuration-startTime)/(numMotionPlanPositionSamples_-1);
  const double splineArrowDt = (splineDuration-startTime)/(numMotionPlanArrowSamples_-1);
  const ros::Time stamp = ros::Time::now();

  // Helper vectors
  loco::Position           cogPos, pathRegPos, initialPos, finalPos, zmpPos, zMinBoud, zMaxBound;
  loco::LinearVelocity     cogVel, pathRegVel, initialVel, finalVel;
  loco::LinearAcceleration cogAcc, pathRegAcc, initialAcc, finalAcc;
  loco::EulerAnglesZyx     cogAngle, pathAngle;
  loco::EulerAnglesZyxDiff cogEulerVel;


  // Initial state.
  if (plannedInitialState_.first.getNumSubscribers() > 0u) {
    motionPlan.getWorldToInitialPositionInWorldFrame(initialPos);
    motionPlan.getInitialVelocityInWorldFrame(initialVel);
    motionPlan.getInitialAccelerationInWorldFrame(initialAcc);

    if (visualizeOnGround_) {
      initialPos = motionPlan.getVirtualPlaneFrame().projectOntoVirtualPlaneInWorldFrame(initialPos);
    }

    plannedInitialState_.second.markers[0].pose.position.x = initialPos.x();
    plannedInitialState_.second.markers[0].pose.position.y = initialPos.y();
    plannedInitialState_.second.markers[0].pose.position.z = initialPos.z();

    plannedInitialState_.second.markers[1].points[0] = plannedInitialState_.second.markers[0].pose.position;
    plannedInitialState_.second.markers[1].points[1].x = plannedInitialState_.second.markers[1].points[0].x + initialVel.x()*scaleVelocity;
    plannedInitialState_.second.markers[1].points[1].y = plannedInitialState_.second.markers[1].points[0].y + initialVel.y()*scaleVelocity;
    plannedInitialState_.second.markers[1].points[1].z = plannedInitialState_.second.markers[1].points[0].z + initialVel.z()*scaleVelocity;

    plannedInitialState_.second.markers[2].points[0] = plannedInitialState_.second.markers[0].pose.position;
    plannedInitialState_.second.markers[2].points[1].x = plannedInitialState_.second.markers[2].points[0].x + initialAcc.x()*scaleAcceleration;
    plannedInitialState_.second.markers[2].points[1].y = plannedInitialState_.second.markers[2].points[0].y + initialAcc.y()*scaleAcceleration;
    plannedInitialState_.second.markers[2].points[1].z = plannedInitialState_.second.markers[2].points[0].z + initialAcc.z()*scaleAcceleration;
  }

  // Final state.
  if (plannedFinalState_.first.getNumSubscribers() > 0u) {
    motionPlan.getWorldToFinalPositionInWorldFrame(finalPos);
    motionPlan.getFinalVelocityInWorldFrame(finalVel);
    motionPlan.getFinalAccelerationInWorldFrame(finalAcc);

    if (visualizeOnGround_) {
      finalPos = motionPlan.getVirtualPlaneFrame().projectOntoVirtualPlaneInWorldFrame(finalPos);
    }

    plannedFinalState_.second.markers[0].pose.position.x = finalPos.x();
    plannedFinalState_.second.markers[0].pose.position.y = finalPos.y();
    plannedFinalState_.second.markers[0].pose.position.z = finalPos.z();

    plannedFinalState_.second.markers[1].points[0] = plannedFinalState_.second.markers[0].pose.position;
    plannedFinalState_.second.markers[1].points[1].x = plannedFinalState_.second.markers[1].points[0].x + finalVel.x()*scaleVelocity;
    plannedFinalState_.second.markers[1].points[1].y = plannedFinalState_.second.markers[1].points[0].y + finalVel.y()*scaleVelocity;
    plannedFinalState_.second.markers[1].points[1].z = plannedFinalState_.second.markers[1].points[0].z + finalVel.z()*scaleVelocity;

    plannedFinalState_.second.markers[2].points[0] = plannedFinalState_.second.markers[0].pose.position;
    plannedFinalState_.second.markers[2].points[1].x = plannedFinalState_.second.markers[2].points[0].x + finalAcc.x()*scaleAcceleration;
    plannedFinalState_.second.markers[2].points[1].y = plannedFinalState_.second.markers[2].points[0].y + finalAcc.y()*scaleAcceleration;
    plannedFinalState_.second.markers[2].points[1].z = plannedFinalState_.second.markers[2].points[0].z + finalAcc.z()*scaleAcceleration;
  }

  // Position Samples.
  double splineTime = startTime;
  for (unsigned int k=0u; k<numMotionPlanPositionSamples_; ++k) {

    // which color is to be used
    const unsigned int activePolygonId = motionPlan.computeActivePolygonIdAtTime(splineTime);
    const loco::Vector& colors = ((motionPlan.getSupportPolygonsInPlaneFrame().size()>0 && motionPlan.getSupportPolygonsInPlaneFrame()[activePolygonId].getPolygonType() == zmp::PolygonType::Empty) ?
        colorsFlightPhase_ : colors_[robot_utils::intmod(activePolygonId+colorId_, colors_.size())]);

    // Torso position
    if (isCogPlan && plannedComPosition_.first.getNumSubscribers() > 0u) {
      cogPos = motionPlan.getPositionWorldToComInWorldFrameAtTime(splineTime);
      plannedComPosition_.second.markers[k].header.stamp = stamp;
      plannedComPosition_.second.markers[k].pose.position.x = cogPos.x();
      plannedComPosition_.second.markers[k].pose.position.y = cogPos.y();
      plannedComPosition_.second.markers[k].pose.position.z = cogPos.z();

      plannedComPosition_.second.markers[k].color.r = colors.x();
      plannedComPosition_.second.markers[k].color.g = colors.y();
      plannedComPosition_.second.markers[k].color.b = colors.z();
    }

    if (isCogPlan && plannedComPositionOnPlane_.first.getNumSubscribers() > 0u) {
      if (plannedComPosition_.first.getNumSubscribers() == 0u) {
        cogPos = motionPlan.getPositionWorldToComInWorldFrameAtTime(splineTime);
      }
      cogPos = motionPlan.getVirtualPlaneFrame().projectOntoVirtualPlaneAlongPlaneNormalInWorldFrame(cogPos);
      plannedComPositionOnPlane_.second.markers[k].header.stamp = stamp;
      plannedComPositionOnPlane_.second.markers[k].pose.position.x = cogPos.x();
      plannedComPositionOnPlane_.second.markers[k].pose.position.y = cogPos.y();
      plannedComPositionOnPlane_.second.markers[k].pose.position.z = cogPos.z();

      plannedComPositionOnPlane_.second.markers[k].color.r = colors.x();
      plannedComPositionOnPlane_.second.markers[k].color.g = colors.y();
      plannedComPositionOnPlane_.second.markers[k].color.b = colors.z();
    }

    // Path regularizer position
    if (isPathRegularizer && pathRegularizer_.first.getNumSubscribers() > 0u) {
      pathRegPos = motionPlan.getPositionWorldToPathInWorldFrameAtTime(splineTime);
      pathRegularizer_.second.markers[k].header.stamp = stamp;
      pathRegularizer_.second.markers[k].pose.position.x = pathRegPos.x();
      pathRegularizer_.second.markers[k].pose.position.y = pathRegPos.y();
      pathRegularizer_.second.markers[k].pose.position.z = pathRegPos.z();

      pathRegularizer_.second.markers[k].color.r = colors.x();
      pathRegularizer_.second.markers[k].color.g = colors.y();
      pathRegularizer_.second.markers[k].color.b = colors.z();
    }

    if (isPathRegularizer && pathRegularizerOnPlane_.first.getNumSubscribers() > 0u) {
      if (pathRegularizer_.first.getNumSubscribers() == 0u) {
        pathRegPos = motionPlan.getPositionWorldToPathInWorldFrameAtTime(splineTime);
      }
      pathRegPos = motionPlan.getVirtualPlaneFrame().projectOntoVirtualPlaneAlongPlaneNormalInWorldFrame(pathRegPos);
      pathRegularizerOnPlane_.second.markers[k].header.stamp = stamp;
      pathRegularizerOnPlane_.second.markers[k].pose.position.x = pathRegPos.x();
      pathRegularizerOnPlane_.second.markers[k].pose.position.y = pathRegPos.y();
      pathRegularizerOnPlane_.second.markers[k].pose.position.z = pathRegPos.z();

      pathRegularizerOnPlane_.second.markers[k].color.r = colors.x();
      pathRegularizerOnPlane_.second.markers[k].color.g = colors.y();
      pathRegularizerOnPlane_.second.markers[k].color.b = colors.z();
    }

    // zmp position
    if (isCogPlan && plannedZmpPosition_.first.getNumSubscribers() > 0u) {
      motionPlan.getWorldToZmpPositionInWorldFrameAtTime(zmpPos, splineTime);
      plannedZmpPosition_.second.markers[k].header.stamp = stamp;
      plannedZmpPosition_.second.markers[k].pose.position.x = zmpPos.x();
      plannedZmpPosition_.second.markers[k].pose.position.y = zmpPos.y();
      plannedZmpPosition_.second.markers[k].pose.position.z = zmpPos.z();

      plannedZmpPosition_.second.markers[k].color.r = colors.x();
      plannedZmpPosition_.second.markers[k].color.g = colors.y();
      plannedZmpPosition_.second.markers[k].color.b = colors.z();

//      unsigned int polygonId = motionPlan.computeActivePolygonIdAtTime(splineTime);
//      if (motionPlan.getSupportPolygonsInPlaneFrame()[polygonId].getPolygonType() != zmp::PolygonType::Empty) {
//
//      }

    }

    splineTime += splineDt;
  }

  // Arrow Samples.
  splineTime = startTime;
  for (unsigned int k=0u; k<numMotionPlanArrowSamples_; ++k) {

    // which color is to be used
    const unsigned int activePolygonId = motionPlan.computeActivePolygonIdAtTime(splineTime);
    const loco::Vector& colors =  ((motionPlan.getSupportPolygonsInPlaneFrame().size()>0 && motionPlan.getSupportPolygonsInPlaneFrame()[activePolygonId].getPolygonType() == zmp::PolygonType::Empty) ?
        colorsFlightPhase_ : colors_[robot_utils::intmod(activePolygonId+colorId_, colors_.size())]);

    // Velocity.
    if (isCogPlan && plannedComVelocity_.first.getNumSubscribers() > 0u) {
      cogVel = motionPlan.getLinearVelocityComInWorldFrameAtTime(splineTime);
      plannedComVelocity_.second.markers[k].header.stamp = stamp;
      plannedComVelocity_.second.markers[k].points[0] = plannedComPosition_.second.markers[ratio_*k].pose.position;
      plannedComVelocity_.second.markers[k].points[1].x = plannedComVelocity_.second.markers[k].points[0].x + scaleVelocity*cogVel.x();
      plannedComVelocity_.second.markers[k].points[1].y = plannedComVelocity_.second.markers[k].points[0].y + scaleVelocity*cogVel.y();
      plannedComVelocity_.second.markers[k].points[1].z = plannedComVelocity_.second.markers[k].points[0].z + scaleVelocity*cogVel.z();

      plannedComVelocity_.second.markers[k].color.r = colors.x();
      plannedComVelocity_.second.markers[k].color.g = colors.y();
      plannedComVelocity_.second.markers[k].color.b = colors.z();
    }

    // Acceleration.
    if (isCogPlan && plannedComAcceleration_.first.getNumSubscribers() > 0u) {
      cogAcc = motionPlan.getLinearAccelerationComInWorldFrameAtTime(splineTime);
      plannedComAcceleration_.second.markers[k].header.stamp = stamp;
      plannedComAcceleration_.second.markers[k].points[0] = plannedComPosition_.second.markers[ratio_*k].pose.position;
      plannedComAcceleration_.second.markers[k].points[1].x = plannedComAcceleration_.second.markers[k].points[0].x + scaleAcceleration*cogAcc.x();
      plannedComAcceleration_.second.markers[k].points[1].y = plannedComAcceleration_.second.markers[k].points[0].y + scaleAcceleration*cogAcc.y();
      plannedComAcceleration_.second.markers[k].points[1].z = plannedComAcceleration_.second.markers[k].points[0].z + scaleAcceleration*cogAcc.z();

      plannedComAcceleration_.second.markers[k].color.r = colors.x();
      plannedComAcceleration_.second.markers[k].color.g = colors.y();
      plannedComAcceleration_.second.markers[k].color.b = colors.z();
    }

    // Orientation.
    if (isCogPlan && plannedComOrientation_.first.getNumSubscribers() > 0u) {
      // orientationDesiredBaseToControl
      loco::RotationQuaternion orientationDesiredBaseToWorld(motionPlan.getAnglesZyxWorldToBaseAtTime(splineTime).inverted().getUnique());
      geometry_msgs::Pose pose;
      pose.position = plannedComPosition_.second.markers[ratio_*k].pose.position;
      pose.orientation.w = orientationDesiredBaseToWorld.w();
      pose.orientation.x = orientationDesiredBaseToWorld.x();
      pose.orientation.y = orientationDesiredBaseToWorld.y();
      pose.orientation.z = orientationDesiredBaseToWorld.z();
      plannedComOrientation_.second.poses[k] = pose;
    }

    if (isCogPlan && plannedComEulerVel_.first.getNumSubscribers() > 0u) {
      const auto orientationPlaneToControl = motionPlan.getOrientationWorldToControl() * motionPlan.getVirtualPlaneFrame().getPosePlaneToWorld().getRotation();
      const auto eulerRatesInPlaneFrame = motionPlan.getComStateInPlaneFrame().getEulerRatesZyxBaseInPlaneFrameAtTime(splineTime);

      // For better visualization: rotate into world frame.
      Eigen::Vector3d cogRatesRotated(eulerRatesInPlaneFrame.roll(), eulerRatesInPlaneFrame.pitch(), eulerRatesInPlaneFrame.yaw());
      cogRatesRotated = orientationPlaneToControl.rotate(cogRatesRotated);

      plannedComEulerVel_.second.markers[k].header.stamp = stamp;
      plannedComEulerVel_.second.markers[k].points[0] = plannedComPosition_.second.markers[ratio_*k].pose.position;
      plannedComEulerVel_.second.markers[k].points[1].x = plannedComEulerVel_.second.markers[k].points[0].x - scaleOrientation*cogRatesRotated.x();
      plannedComEulerVel_.second.markers[k].points[1].y = plannedComEulerVel_.second.markers[k].points[0].y - scaleOrientation*cogRatesRotated.y();
      plannedComEulerVel_.second.markers[k].points[1].z = plannedComEulerVel_.second.markers[k].points[0].z - scaleOrientation*cogRatesRotated.z();

      plannedComEulerVel_.second.markers[k].color.r = colors.x();
      plannedComEulerVel_.second.markers[k].color.g = colors.y();
      plannedComEulerVel_.second.markers[k].color.b = colors.z();
    }

    if (isPathRegularizer && pathRegularizerOrientation_.first.getNumSubscribers() > 0u) {
      const loco::RotationQuaternion orientationPathToWorld(motionPlan.getAnglesZyxWorldToPathAtTime(splineTime).inverted().getUnique());
      geometry_msgs::Pose pose;
      pose.position = pathRegularizer_.second.markers[ratio_*k].pose.position;
      pose.orientation.w = orientationPathToWorld.w();
      pose.orientation.x = orientationPathToWorld.x();
      pose.orientation.y = orientationPathToWorld.y();
      pose.orientation.z = orientationPathToWorld.z();
      pathRegularizerOrientation_.second.poses[k] = pose;
    }

    splineTime += splineArrowDt;
  }


  // Line strip markers for cog
  if (isCogPlan && plannedComPositionLineStrip_.first.getNumSubscribers() > 0u) {
    plannedComPositionLineStrip_.second.points.clear();
    plannedComPositionLineStrip_.second.points.reserve(plannedComPosition_.second.markers.size());
    for (const auto& marker : plannedComPosition_.second.markers) {
      plannedComPositionLineStrip_.second.points.push_back(marker.pose.position);
    }
    plannedComPositionLineStrip_.second.header.stamp = stamp;
  }

  // line strip markers for zmp
  if (isCogPlan && plannedZmpPositionLineStrip_.first.getNumSubscribers() > 0u) {
    plannedZmpPositionLineStrip_.second.points.clear();
    plannedZmpPositionLineStrip_.second.points.reserve(plannedZmpPosition_.second.markers.size());
    for (const auto& marker : plannedZmpPosition_.second.markers) {
      plannedZmpPositionLineStrip_.second.points.push_back(marker.pose.position);
    }
    plannedZmpPositionLineStrip_.second.header.stamp = stamp;
  }

  return true;
}

bool MotionPlanVisualizer::publish() {
  loco_ros::publishMsg(plannedComPosition_);
  loco_ros::publishMsg(plannedComPositionOnPlane_);
  loco_ros::publishMsg(plannedComPositionLineStrip_);
  loco_ros::publishMsg(plannedComVelocity_);
  loco_ros::publishMsg(plannedComAcceleration_);
  loco_ros::publishMsg(plannedComOrientation_);
  loco_ros::publishMsg(plannedComEulerVel_);
  loco_ros::publishMsg(pathRegularizerOrientation_);
  loco_ros::publishMsg(plannedZmpPosition_);
  loco_ros::publishMsg(plannedZmpPositionLineStrip_);
  loco_ros::publishMsg(pathRegularizer_);
  loco_ros::publishMsg(pathRegularizerOnPlane_);
  loco_ros::publishMsg(plannedInitialState_);
  loco_ros::publishMsg(plannedFinalState_);
  return true;
}

void MotionPlanVisualizer::setVisualizeOnGround(bool visualizeOnGround) {
  visualizeOnGround_ = visualizeOnGround;
}

} /* namespace loco */
