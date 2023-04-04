/*
 * Torso.cpp
 *
 *  Created on: Sep 3, 2015
 *      Author: Christian Gehring, Peter Fankhauser
 */
#include "loco_ros/visualization/Torso.hpp"

namespace loco_ros {

Torso::Torso() {
  publisherRefs_.push_back(publisher_);
  publisherRefs_.push_back(desiredBaseLinearVelocityPub_);
  publisherRefs_.push_back(desiredBaseAngularVelocityPub_);
  publisherRefs_.push_back(desiredBaseLinearAccelerationPub_);
  publisherRefs_.push_back(commandedBaseLinearVelocityPub_);
  publisherRefs_.push_back(commandedBaseAngularVelocityPub_);

  markerArray_.markers.resize(5, visualization_msgs::Marker());
  measBaseSphere_ = &markerArray_.markers[0];
  measBaseLine_ = &markerArray_.markers[1];
  desBaseSphere_ = &markerArray_.markers[2];
  desBaseLine_ = &markerArray_.markers[3];
  measBaseOnTerrain_ = &markerArray_.markers[4];
}

bool Torso::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  publisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topic, 1);

  measBaseSphere_->pose.orientation.w = 1.0;
  measBaseSphere_->action = visualization_msgs::Marker::ADD;
  measBaseSphere_->type = visualization_msgs::Marker::SPHERE;
  measBaseSphere_->scale.x = 0.03;
  measBaseSphere_->scale.y = 0.03;
  measBaseSphere_->scale.z = 0.03;
  measBaseSphere_->color.a = 1.0;
  measBaseSphere_->color.r = 1.0;
  measBaseSphere_->color.g = 1.0;
  measBaseSphere_->color.b = 0.0;
  measBaseSphere_->id = 1;
  measBaseSphere_->header.frame_id = "odom";
  measBaseSphere_->ns = "meas_base_sphere";

  desBaseSphere_->pose.orientation.w = 1.0;
  desBaseSphere_->action = visualization_msgs::Marker::ADD;
  desBaseSphere_->type = visualization_msgs::Marker::SPHERE;
  desBaseSphere_->scale.x = 0.03;
  desBaseSphere_->scale.y = 0.03;
  desBaseSphere_->scale.z = 0.03;
  desBaseSphere_->color.a = 1.0;
  desBaseSphere_->color.r = 1.0;
  desBaseSphere_->color.g = 0.0;
  desBaseSphere_->color.b = 0.0;
  desBaseSphere_->id = 3;
  desBaseSphere_->header.frame_id = "odom";
  desBaseSphere_->ns = "des_base_sphere";


  measBaseLine_->pose.orientation.w = 1.0;
  measBaseLine_->action = visualization_msgs::Marker::ADD;
  measBaseLine_->type = visualization_msgs::Marker::LINE_LIST;
  measBaseLine_->scale.x = 0.005;
  measBaseLine_->scale.y = 0.005;
  measBaseLine_->color.a = 1.0;
  measBaseLine_->color.r = 1.0;
  measBaseLine_->color.g = 1.0;
  measBaseLine_->color.b = 0.0;
  measBaseLine_->id = 2;
  measBaseLine_->header.frame_id = "odom";
  measBaseLine_->ns = "meas_base_line";
  measBaseLine_->points.clear();
  measBaseLine_->points.resize(2,geometry_msgs::Point());


  desBaseLine_->pose.orientation.w = 1.0;
  desBaseLine_->action = visualization_msgs::Marker::ADD;
  desBaseLine_->type = visualization_msgs::Marker::LINE_LIST;
  desBaseLine_->scale.x = 0.005;
  desBaseLine_->scale.y = 0.005;
  desBaseLine_->color.a = 1.0;
  desBaseLine_->color.r = 1.0;
  desBaseLine_->color.g = 0.0;
  desBaseLine_->color.b = 0.0;
  desBaseLine_->id = 4;
  desBaseLine_->header.frame_id = "odom";
  desBaseLine_->ns = "des_base_line";
  desBaseLine_->points.clear();
  desBaseLine_->points.resize(2,geometry_msgs::Point());


  measBaseOnTerrain_->pose.orientation.w = 1.0;
  measBaseOnTerrain_->action = visualization_msgs::Marker::ADD;
  measBaseOnTerrain_->type = visualization_msgs::Marker::SPHERE;
  measBaseOnTerrain_->scale.x = 0.015;
  measBaseOnTerrain_->scale.y = 0.015;
  measBaseOnTerrain_->scale.z = 0.015;
  measBaseOnTerrain_->color.a = 1.0;
  measBaseOnTerrain_->color.r = 1.0;
  measBaseOnTerrain_->color.g = 1.0;
  measBaseOnTerrain_->color.b = 0.0;
  measBaseOnTerrain_->id = 1;
  measBaseOnTerrain_->header.frame_id = "odom";
  measBaseOnTerrain_->ns = "meas_base_sphere_on_terrain";

  // Desired main body motion (velocity and acceleration).
  desiredBaseLinearVelocityMsg_.header.frame_id = "control";
  desiredBaseLinearVelocityMsg_.position_frame_id = "odom";
  desiredBaseLinearVelocityMsg_.type = kindr_msgs::VectorAtPosition::TYPE_VELOCITY;

  desiredBaseAngularVelocityMsg_.header.frame_id = "control";
  desiredBaseAngularVelocityMsg_.position_frame_id = "odom";
  desiredBaseAngularVelocityMsg_.type = kindr_msgs::VectorAtPosition::TYPE_ANGULAR_VELOCITY;

  desiredBaseLinearAccelerationMsg_.header.frame_id = "control";
  desiredBaseLinearAccelerationMsg_.position_frame_id = "odom";
  desiredBaseLinearAccelerationMsg_.type = kindr_msgs::VectorAtPosition::TYPE_ACCELERATION;

  commandedBaseLinearVelocityMsg_.header.frame_id = "control";
  commandedBaseLinearVelocityMsg_.position_frame_id = "odom";
  commandedBaseLinearVelocityMsg_.type = kindr_msgs::VectorAtPosition::TYPE_VELOCITY;

  commandedBaseAngularVelocityMsg_.header.frame_id = "control";
  commandedBaseAngularVelocityMsg_.position_frame_id = "odom";
  commandedBaseAngularVelocityMsg_.type = kindr_msgs::VectorAtPosition::TYPE_ANGULAR_VELOCITY;

  desiredBaseLinearVelocityPub_ = nodeHandle.advertise<kindr_msgs::VectorAtPosition>("/loco_ros/desired_torso_linear_velocity", 1);
  desiredBaseAngularVelocityPub_ = nodeHandle.advertise<kindr_msgs::VectorAtPosition>("/loco_ros/desired_torso_angular_velocity", 1);
  desiredBaseLinearAccelerationPub_ = nodeHandle.advertise<kindr_msgs::VectorAtPosition>("/loco_ros/desired_torso_acceleration", 1);
  commandedBaseLinearVelocityPub_ = nodeHandle.advertise<kindr_msgs::VectorAtPosition>("/loco_ros/commanded_torso_linear_velocity", 1);
  commandedBaseAngularVelocityPub_ = nodeHandle.advertise<kindr_msgs::VectorAtPosition>("/loco_ros/commanded_torso_angular_velocity", 1);

  return true;
}

bool Torso::update(const loco::TorsoBase& torso, const loco::Legs& legs, const loco::TerrainModelBase& terrainModel) {
  if (!(getNumSubscribers() > 0u)) {
    return true;
  }

  const ros::Time timestamp = ros::Time::now();

  if (publisher_.getNumSubscribers() > 0u) {
    ros::Time stamp = ros::Time::now();
    measBaseSphere_->header.stamp = stamp;
    measBaseLine_->header.stamp = stamp;

    const loco::Position& position = torso.getMeasuredState().getPositionWorldToBaseInWorldFrame();
    measBaseSphere_->pose.position.x = position.x();
    measBaseSphere_->pose.position.y = position.y();
    measBaseSphere_->pose.position.z = position.z();

    measBaseOnTerrain_->pose.position.x = position.x();
    measBaseOnTerrain_->pose.position.y = position.y();
    terrainModel.getHeight(position, measBaseOnTerrain_->pose.position.z);

    measBaseLine_->points[0].x = position.x();
    measBaseLine_->points[0].y = position.y();
    measBaseLine_->points[0].z = 0.0;
    measBaseLine_->points[1].x = position.x();
    measBaseLine_->points[1].y = position.y();
    measBaseLine_->points[1].z = position.z();

    const loco::Position& desPosition = torso.getDesiredState().getPositionWorldToBaseInWorldFrame();
    desBaseSphere_->pose.position.x = desPosition.x();
    desBaseSphere_->pose.position.y = desPosition.y();
    desBaseSphere_->pose.position.z = desPosition.z();

    desBaseLine_->points[0].x = desPosition.x();
    desBaseLine_->points[0].y = desPosition.y();
    desBaseLine_->points[0].z = 0.0;
    desBaseLine_->points[1].x = desPosition.x();
    desBaseLine_->points[1].y = desPosition.y();
    desBaseLine_->points[1].z = desPosition.z();
  }

  // get the average feet height to offset the markers
  double supportFootprintHeight = 0;
  unsigned int numSupportLegs = 0;
  for (const auto& leg : legs) {
    if ( (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) ||
         (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant) ) {
      supportFootprintHeight += leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame().z();
      numSupportLegs++;
    }
  }
  if (numSupportLegs > 0) {
    supportFootprintHeight /= (double)numSupportLegs;
  }

  // get rotations
  const loco::RotationQuaternion& orientationControlToDesiredBase = torso.getDesiredState().getOrientationControlToBase();
  const loco::RotationQuaternion& orientationWorldToControl = torso.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const loco::RotationQuaternion orientationWorldToDesiredBase = orientationControlToDesiredBase*orientationWorldToControl;

  // get position vectors
  const loco::Position& positionControlToDesiredBaseInControlFrame = torso.getDesiredState().getPositionControlToTargetInControlFrame();
  const loco::Position& positionWorldToControlInWorldFrame = torso.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();
  const loco::Position positionWorldToDesiredBaseInWorldFrame = positionWorldToControlInWorldFrame
                                                              + orientationWorldToControl.inverseRotate(positionControlToDesiredBaseInControlFrame);

  const loco::Position& positionWorldToBaseInWorldFrame = torso.getMeasuredState().getPositionWorldToBaseInWorldFrame();

  if (desiredBaseLinearVelocityPub_.getNumSubscribers() > 0u) {
    const loco::LinearVelocity& desiredBaseLinearVelocityInControlFrame = torso.getDesiredState().getLinearVelocityTargetInControlFrame();
    desiredBaseLinearVelocityMsg_.position.x = positionWorldToDesiredBaseInWorldFrame.x();
    desiredBaseLinearVelocityMsg_.position.y = positionWorldToDesiredBaseInWorldFrame.y();
    desiredBaseLinearVelocityMsg_.position.z = supportFootprintHeight;
    desiredBaseLinearVelocityMsg_.vector.x = desiredBaseLinearVelocityInControlFrame.x();
    desiredBaseLinearVelocityMsg_.vector.y = desiredBaseLinearVelocityInControlFrame.y();
    desiredBaseLinearVelocityMsg_.vector.z = desiredBaseLinearVelocityInControlFrame.z();
  }

  if (desiredBaseAngularVelocityPub_.getNumSubscribers() > 0u) {
    const loco::LocalAngularVelocity& desiredAngularBaseVelocityInControlFrame = torso.getDesiredState().getAngularVelocityBaseInControlFrame();
    desiredBaseAngularVelocityMsg_.position.x = positionWorldToDesiredBaseInWorldFrame.x();
    desiredBaseAngularVelocityMsg_.position.y = positionWorldToDesiredBaseInWorldFrame.y();
    desiredBaseAngularVelocityMsg_.position.z = supportFootprintHeight;
    desiredBaseAngularVelocityMsg_.vector.x = desiredAngularBaseVelocityInControlFrame.x();
    desiredBaseAngularVelocityMsg_.vector.y = desiredAngularBaseVelocityInControlFrame.y();
    desiredBaseAngularVelocityMsg_.vector.z = desiredAngularBaseVelocityInControlFrame.z();
  }

  if (desiredBaseLinearAccelerationPub_.getNumSubscribers() > 0u) {
    const loco::LinearAcceleration& desiredBaseLinearAccelerationInControlFrame = torso.getDesiredState().getLinearAccelerationTargetInControlFrame();
    desiredBaseLinearAccelerationMsg_.position.x = positionWorldToDesiredBaseInWorldFrame.x();
    desiredBaseLinearAccelerationMsg_.position.y = positionWorldToDesiredBaseInWorldFrame.y();
    desiredBaseLinearAccelerationMsg_.position.z = supportFootprintHeight;
    desiredBaseLinearAccelerationMsg_.vector.x = desiredBaseLinearAccelerationInControlFrame.x();
    desiredBaseLinearAccelerationMsg_.vector.y = desiredBaseLinearAccelerationInControlFrame.y();
    desiredBaseLinearAccelerationMsg_.vector.z = desiredBaseLinearAccelerationInControlFrame.z();
  }

  if (commandedBaseLinearVelocityPub_.getNumSubscribers() > 0u) {
    const loco::LinearVelocity& commandedBaseLinearVelocityInControlFrame = torso.getDesiredState().getLinearVelocityCommandedTargetInControlFrame();
    commandedBaseLinearVelocityMsg_.position.x = positionWorldToDesiredBaseInWorldFrame.x();
    commandedBaseLinearVelocityMsg_.position.y = positionWorldToDesiredBaseInWorldFrame.y();
    commandedBaseLinearVelocityMsg_.position.z = supportFootprintHeight;
    commandedBaseLinearVelocityMsg_.vector.x = commandedBaseLinearVelocityInControlFrame.x();
    commandedBaseLinearVelocityMsg_.vector.y = commandedBaseLinearVelocityInControlFrame.y();
    commandedBaseLinearVelocityMsg_.vector.z = commandedBaseLinearVelocityInControlFrame.z();
  }

  if (commandedBaseAngularVelocityPub_.getNumSubscribers() > 0u) {
    const loco::LocalAngularVelocity& commandedAngularBaseVelocityInControlFrame = torso.getDesiredState().getAngularVelocityCommandedBaseInControlFrame();
    commandedBaseAngularVelocityMsg_.position.x = positionWorldToDesiredBaseInWorldFrame.x();
    commandedBaseAngularVelocityMsg_.position.y = positionWorldToDesiredBaseInWorldFrame.y();
    commandedBaseAngularVelocityMsg_.position.z = supportFootprintHeight;
    commandedBaseAngularVelocityMsg_.vector.x = commandedAngularBaseVelocityInControlFrame.x();
    commandedBaseAngularVelocityMsg_.vector.y = commandedAngularBaseVelocityInControlFrame.y();
    commandedBaseAngularVelocityMsg_.vector.z = commandedAngularBaseVelocityInControlFrame.z();
  }

  return true;
}

bool Torso::publish() {
  loco_ros::publishMsg(publisher_, markerArray_);

  publishMsg<kindr_msgs::VectorAtPosition>(desiredBaseLinearVelocityPub_, desiredBaseLinearVelocityMsg_);
  publishMsg<kindr_msgs::VectorAtPosition>(desiredBaseAngularVelocityPub_, desiredBaseAngularVelocityMsg_);
  publishMsg<kindr_msgs::VectorAtPosition>(desiredBaseLinearAccelerationPub_, desiredBaseLinearAccelerationMsg_);
  publishMsg<kindr_msgs::VectorAtPosition>(commandedBaseLinearVelocityPub_, commandedBaseLinearVelocityMsg_);
  publishMsg<kindr_msgs::VectorAtPosition>(commandedBaseAngularVelocityPub_, commandedBaseAngularVelocityMsg_);

  return true;
}

} /* namespace loco_ros */
