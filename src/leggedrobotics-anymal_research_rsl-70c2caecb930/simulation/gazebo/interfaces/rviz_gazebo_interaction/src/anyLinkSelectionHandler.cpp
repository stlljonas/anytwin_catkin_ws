/*!
* @file     anyLinkSelectionHandler.cpp
* @author   Linus Isler
* @date     October, 2016
*/

#include "rviz_gazebo_interaction/anyLinkSelectionHandler.hpp"
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_joint.h>

namespace rviz_gazebo_interaction {

AnyLinkSelectionHandler::AnyLinkSelectionHandler(rviz::RobotLink* link, rviz::DisplayContext* context)
  : RobotLinkSelectionHandler(link, context)
{

}


AnyLinkSelectionHandler::~AnyLinkSelectionHandler() { }


bool AnyLinkSelectionHandler::isRobotLink() {
  if (link_) {
    return true;
  }
  return false;
}


const std::string& AnyLinkSelectionHandler::getLinkName() const {
  return link_->getName();
}

const std::string AnyLinkSelectionHandler::getCollisionParentLinkName(const std::vector<std::string>& collNames) const {
  if (std::find(collNames.begin(), collNames.end(), "anymal::" + link_->getName()) != collNames.end()) {
    return link_->getName();
  }
  rviz::Robot* robot = link_->getRobot();
  rviz::RobotLink* parentLink = nullptr;
  if (robot != nullptr) {
    rviz::RobotJoint* parentJoint = robot->getJoint(link_->getParentJointName());
    while (parentJoint != nullptr) {
      parentLink = robot->getLink(parentJoint->getParentLinkName());
      if (parentLink != nullptr) {
        if (std::find(collNames.begin(), collNames.end(), "anymal::" + parentLink->getName()) != collNames.end()) {
          break;
        }
        parentJoint = robot->getJoint(parentLink->getParentJointName());
      } else {
        break;
      }
    }
  }
  return parentLink != nullptr ? parentLink->getName() : link_->getName();
}

void AnyLinkSelectionHandler::printLinkName() {
  ROS_INFO_STREAM(link_->getName());
}

const Ogre::Vector3 AnyLinkSelectionHandler::getLinkPosition() const {
  return link_->getPosition();
}

} //namespace rviz_gazebo_interaction

