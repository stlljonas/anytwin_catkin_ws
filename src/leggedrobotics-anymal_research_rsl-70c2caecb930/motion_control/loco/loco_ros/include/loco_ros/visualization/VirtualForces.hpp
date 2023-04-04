/*
 * VirtualForces.hpp
 *
 *  Created on: Jan 17, 2017
 *      Author: gech
 */

#pragma once

#include <loco/common/torso/TorsoBase.hpp>
#include <loco/motion_control/VirtualModelController.hpp>

#include <loco_ros/loco_ros.hpp>

namespace loco_ros {

class VirtualForces {
 public:
  VirtualForces() {};
  virtual ~VirtualForces() {};
  bool initialize(ros::NodeHandle& nodeHandle,
                  const std::string& baseFrameId = "base")
  {
    virtualForce_.second.header.frame_id = baseFrameId;
    virtualForce_.second.position_frame_id = "odom";
    virtualForce_.second.type = kindr_msgs::VectorAtPosition::TYPE_FORCE;

    virtualTorque_.second.header.frame_id = baseFrameId;
    virtualTorque_.second.position_frame_id = "odom";
    virtualTorque_.second.type = kindr_msgs::VectorAtPosition::TYPE_TORQUE;

    netVirtualForce_.second.header.frame_id = baseFrameId;
    netVirtualForce_.second.position_frame_id = "odom";
    netVirtualForce_.second.type = kindr_msgs::VectorAtPosition::TYPE_FORCE;

    netVirtualTorque_.second.header.frame_id = baseFrameId;
    netVirtualTorque_.second.position_frame_id = "odom";
    netVirtualTorque_.second.type = kindr_msgs::VectorAtPosition::TYPE_TORQUE;

    // Virtual wrench.
    virtualForce_.first = nodeHandle_.advertise<decltype(virtualForce_.second)>("/loco_ros/virtual_force", 1);
    virtualTorque_.first = nodeHandle_.advertise<decltype(virtualTorque_.second)>("/loco_ros/virtual_torque", 1);

    netVirtualForce_.first = nodeHandle_.advertise<decltype(netVirtualForce_.second)>("/loco_ros/net_virtual_force", 1);
    netVirtualTorque_.first = nodeHandle_.advertise<decltype(netVirtualTorque_.second)>("/loco_ros/net_virtual_torque", 1);

    return true;
  }

  bool shutdown() {
    virtualForce_.first.shutdown();
    virtualTorque_.first.shutdown();
    netVirtualForce_.first.shutdown();
    netVirtualTorque_.first.shutdown();
    return true;
  }

  bool update(const loco::Force& desiredForce, const loco::Torque& desiredTorque, const loco::TorsoBase& torso) {
    const loco::Position& positionWorldToBaseInWorldFrame = torso.getMeasuredState().getPositionWorldToBaseInWorldFrame();

    virtualForce_.second.position.x = positionWorldToBaseInWorldFrame.x();
    virtualForce_.second.position.y = positionWorldToBaseInWorldFrame.y();
    virtualForce_.second.position.z = positionWorldToBaseInWorldFrame.z();
    virtualForce_.second.vector.x = desiredForce.x();
    virtualForce_.second.vector.y = desiredForce.y();
    virtualForce_.second.vector.z = desiredForce.z();

    virtualTorque_.second.position.x = positionWorldToBaseInWorldFrame.x();
    virtualTorque_.second.position.y = positionWorldToBaseInWorldFrame.y();
    virtualTorque_.second.position.z = positionWorldToBaseInWorldFrame.z();
    virtualTorque_.second.vector.x = desiredTorque.x();
    virtualTorque_.second.vector.y = desiredTorque.y();
    virtualTorque_.second.vector.z = desiredTorque.z();
    return true;
  }

  bool updateNetForces(const loco::Force& force, const loco::Torque& torque, const loco::TorsoBase& torso) {
    const loco::Position& positionWorldToBaseInWorldFrame = torso.getMeasuredState().getPositionWorldToBaseInWorldFrame();

    netVirtualForce_.second.position.x = positionWorldToBaseInWorldFrame.x();
    netVirtualForce_.second.position.y = positionWorldToBaseInWorldFrame.y();
    netVirtualForce_.second.position.z = positionWorldToBaseInWorldFrame.z();
    netVirtualForce_.second.vector.x = force.x();
    netVirtualForce_.second.vector.y = force.y();
    netVirtualForce_.second.vector.z = force.z();

    netVirtualTorque_.second.position.x = positionWorldToBaseInWorldFrame.x();
    netVirtualTorque_.second.position.y = positionWorldToBaseInWorldFrame.y();
    netVirtualTorque_.second.position.z = positionWorldToBaseInWorldFrame.z();
    netVirtualTorque_.second.vector.x = torque.x();
    netVirtualTorque_.second.vector.y = torque.y();
    netVirtualTorque_.second.vector.z = torque.z();
    return true;
  }

  bool update(const loco::VirtualModelController& controller, const loco::TorsoBase& torso) {
    bool success = update(controller.getDesiredVirtualForceInBaseFrame(), controller.getDesiredVirtualTorqueInBaseFrame(), torso);

    loco::Force netForceInBaseFrame;
    loco::Torque netTorqueInBaseFrame;
    controller.getDistributedVirtualForceAndTorqueInBaseFrame(netForceInBaseFrame, netTorqueInBaseFrame);
    success &= updateNetForces(netForceInBaseFrame, netTorqueInBaseFrame, torso);

    return success;
  }

  bool publish() {
    publishMsg(virtualForce_);
    publishMsg(virtualTorque_);
    publishMsg(netVirtualForce_);
    publishMsg(netVirtualTorque_);
    return true;
  }

protected:
  ros::NodeHandle nodeHandle_;

  std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> virtualForce_;
  std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> virtualTorque_;
  std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> netVirtualForce_;
  std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> netVirtualTorque_;

};

} /* namespace loco_ros */

