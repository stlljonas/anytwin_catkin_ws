/*
 * Ghost.cpp
 *
 *  Created on: Jun 2, 2016
 *      Author: Christian Gehring, C. Dario Bellicoso
 */

#include "loco_ros_anymal/visualization/Ghost.hpp"
#include <loco_ros/loco_ros.hpp>
#include <anymal_model_ros/initializations.hpp>

namespace loco_ros_anymal {

Ghost::Ghost(ghost::StateEnum stateEnum) :
    stateEnum_(stateEnum)
{

}

bool Ghost::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  anymal_model_ros::initialize(ghostAnymalStateMsg_);
  ghostAnymalStateMsg_.frame_transforms.clear();
  ghostAnymalStateMsg_.state = anymal_msgs::AnymalState::STATE_OK;
  ghostAnymalStatePub_ = nodeHandle.advertise<anymal_msgs::AnymalState>(topic, 1);
  return true;
}

bool Ghost::update(const loco::WholeBody& wholeBody, const ros::Time& timestamp) {
  // update the desired anymal state
  if (ghostAnymalStatePub_.getNumSubscribers() > 0u) {
    auto& legs = wholeBody.getLegs();
    auto& torso = wholeBody.getTorso();

     ghostAnymalStateMsg_.joints.header.stamp = timestamp;

     switch (stateEnum_) {
       case(ghost::StateEnum::Desired) : {

         // get rotations
         const loco::RotationQuaternion& orientationControlToDesiredBase = torso.getDesiredState().getOrientationControlToBase();
         const loco::RotationQuaternion& orientationWorldToControl = torso.getMeasuredState().inControlFrame().getOrientationWorldToControl();

         // anymal state needs BaseToWorld
         loco::RotationQuaternion orientationDesiredBaseToWorld = (orientationControlToDesiredBase*orientationWorldToControl).inverted();

         int j = 0;
         for (int k=0; k<legs.size(); ++k) {
           // fixme: remove get
           const loco::JointPositions& jointPositionsDesired = legs.get(k).getLimbStateDesired().getJointPositions();
           const loco::JointVelocities& jointVelocitiesDesired = legs.get(k).getLimbStateDesired().getJointVelocities();
           const loco::JointTorques& jointTorquesDesired = legs.get(k).getLimbStateDesired().getJointTorques();
           for (int i=0; i<jointVelocitiesDesired.size(); ++i) {
             ghostAnymalStateMsg_.joints.position[j] = jointPositionsDesired(i);
             ghostAnymalStateMsg_.joints.velocity[j] = jointVelocitiesDesired(i);
             ghostAnymalStateMsg_.joints.effort[j] = jointTorquesDesired(i);
             ++j;
           }
         }

         // get position vectors
         const loco::Position& positionControlToDesiredBaseInControlFrame = torso.getDesiredState().getPositionControlToTargetInControlFrame();
         const loco::Position& positionWorldToControlInWorldFrame = torso.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();
         const loco::Position positionWorldToDesiredBaseInWorldFrame = positionWorldToControlInWorldFrame
                                                                     + orientationWorldToControl.inverseRotate(positionControlToDesiredBaseInControlFrame);

         ghostAnymalStateMsg_.pose.header.stamp = timestamp;
         ghostAnymalStateMsg_.pose.header.frame_id = "odom";
         ghostAnymalStateMsg_.pose.pose.position.x = positionWorldToDesiredBaseInWorldFrame.x();
         ghostAnymalStateMsg_.pose.pose.position.y = positionWorldToDesiredBaseInWorldFrame.y();
         ghostAnymalStateMsg_.pose.pose.position.z = positionWorldToDesiredBaseInWorldFrame.z();
         ghostAnymalStateMsg_.pose.pose.orientation.w = orientationDesiredBaseToWorld.w();
         ghostAnymalStateMsg_.pose.pose.orientation.x = orientationDesiredBaseToWorld.x();
         ghostAnymalStateMsg_.pose.pose.orientation.y = orientationDesiredBaseToWorld.y();
         ghostAnymalStateMsg_.pose.pose.orientation.z = orientationDesiredBaseToWorld.z();
         ghostAnymalStateMsg_.header.stamp = timestamp;
       } break;

       case(ghost::StateEnum::Measured) : {
         int j = 0;
         for (int k=0; k<legs.size(); ++k) {
           // fixme: remove get
           const loco::JointPositions& jointPositionsMeasured = legs.get(k).getLimbStateMeasured().getJointPositions();
           const loco::JointVelocities& jointVelocitiesMeasured = legs.get(k).getLimbStateMeasured().getJointVelocities();
           const loco::JointTorques& jointTorquesMeasured = legs.get(k).getLimbStateMeasured().getJointTorques();
           for (int i=0; i<jointVelocitiesMeasured.size(); ++i) {
             ghostAnymalStateMsg_.joints.position[j] = jointPositionsMeasured(i);
             ghostAnymalStateMsg_.joints.velocity[j] = jointVelocitiesMeasured(i);
             ghostAnymalStateMsg_.joints.effort[j] = jointTorquesMeasured(i);
             ++j;
           }
         }

         // Get measured pose.
         const loco::Position& positionWorldToMeasuredBaseInWorldFrame = torso.getMeasuredState().getPositionWorldToBaseInWorldFrame();
         const loco::Position& positionWorldToControlInWorldFrame = torso.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();
         const loco::RotationQuaternion& orientationWorldToBase = torso.getMeasuredState().getOrientationWorldToBase();

         ghostAnymalStateMsg_.pose.header.stamp = timestamp;
         ghostAnymalStateMsg_.pose.header.frame_id = "odom";
         ghostAnymalStateMsg_.pose.pose.position.x = positionWorldToMeasuredBaseInWorldFrame.x();
         ghostAnymalStateMsg_.pose.pose.position.y = positionWorldToMeasuredBaseInWorldFrame.y();
         ghostAnymalStateMsg_.pose.pose.position.z = positionWorldToMeasuredBaseInWorldFrame.z();
         ghostAnymalStateMsg_.pose.pose.orientation.w = orientationWorldToBase.w();
         ghostAnymalStateMsg_.pose.pose.orientation.x = -orientationWorldToBase.x();
         ghostAnymalStateMsg_.pose.pose.orientation.y = -orientationWorldToBase.y();
         ghostAnymalStateMsg_.pose.pose.orientation.z = -orientationWorldToBase.z();
         ghostAnymalStateMsg_.header.stamp = timestamp;
       } break;

       default : {
         return false;
       } break;
     }

   }
   return true;
}

bool Ghost::shutdown() {
  ghostAnymalStatePub_.shutdown();
  return true;
}

bool Ghost::publish() {
  loco_ros::publishMsg(ghostAnymalStatePub_, ghostAnymalStateMsg_);
  return true;
}

unsigned int Ghost::getNumSubscribers() const {
  return ghostAnymalStatePub_.getNumSubscribers();
}

} /* namespace loco_ros_anymal */
