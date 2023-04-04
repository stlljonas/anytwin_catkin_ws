/*
 * SimGripper.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: perry
 */

#include <gazebo_gripper/SimGripper.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace gazebo {

SimGripper::SimGripper(const std::string& robotName, physics::ModelPtr& model)
    : robotName_(robotName), model_(model), isAttachedToObject_(false) {
  graspJoint_ = model->GetWorld()->Physics()->CreateJoint("fixed", model);
}

SimGripper::SimGripper(const std::string& robotName, physics::ModelPtr& model, const std::string& collisionLinkName,
                       const std::string& palmLinkName)
    : robotName_(robotName), model_(model), isAttachedToObject_(false) {
  setCollisionLinkName(collisionLinkName);
  setPalmLinkName(palmLinkName);
  graspJoint_ = model->GetWorld()->Physics()->CreateJoint("fixed", model);
}

SimGripper::~SimGripper() {
  graspJoint_.reset();
}

void SimGripper::setCollisionLinkName(const std::string& collisionLinkName) {
  collisionLinkName_ = collisionLinkName;

  linkScopedCollisionNames_.clear();

  auto links = model_->GetLinks();

  std::vector<std::string> collisionScopedNames;

  for (const auto& link : links) {
    const size_t numCollisions = link->GetCollisions().size();

    for (size_t i = 0; i < numCollisions; ++i) {
      const std::string scopedName = link->GetCollision(i)->GetScopedName();

      // The scoped names in gazebo are expected to be ROBOT_NAME::LINK_NAME::LINK_NAME_collision
      //                                       or      ROBOT_NAME::LINK_NAME::LINK_NAME_collision_#
      //                                       or      ROBOT_NAME::LINK_NAME::LINK_NAME_fixed_joint_lump__LINK_NAME_collision_#
      if (scopedName.find(collisionLinkName_ + "_collision") != std::string::npos) {
        linkScopedCollisionNames_.emplace_back(scopedName);
      }
    }
  }

  if (linkScopedCollisionNames_.empty()) {
    MELO_WARN_STREAM("[" << this->robotName_ << "] the named link " << collisionLinkName
                         << " doesn't seem to have any collision geometry...");
  }
}

void SimGripper::setPalmLinkName(const std::string& palmLinkName) {
  palmLink_ = model_->GetLink(palmLinkName);
  if (!palmLink_) {
    MELO_ERROR_STREAM("[" << this->robotName_ << "] palmLink " << palmLinkName << " was not found in the gazebo model; if " << palmLinkName
                          << " is a child of a fixed joint, it might not exist in the gazebo model");
  }
}
std::string SimGripper::getPalmLinkName() const {
  if (!palmLink_) {
    MELO_ERROR_STREAM("[" << this->robotName_ << "] palmLink is null");
    return "";
  }
  return palmLink_->GetName();
}

bool SimGripper::attachGrasp() {
  if (!palmLink_) {
    MELO_WARN_STREAM("[" << this->robotName_ << "] palmLink has not been set");
  }

  if (!isAttachedToObject_) {
    physics::ContactManager* contactManager = model_->GetWorld()->Physics()->GetContactManager();

    physics::LinkPtr graspTargetLinkPtr_;

    // Search through the contacts in the contact manager for collisions involving our collision link
    for (unsigned int contactIndex = 0u; contactIndex < contactManager->GetContactCount(); ++contactIndex) {
      physics::Contact* contact = contactManager->GetContact(contactIndex);

      for (const std::string& linkScopedCollisionName : linkScopedCollisionNames_) {
        if (contact->collision1->GetScopedName() == linkScopedCollisionName) {
          graspTargetLinkPtr_ = contact->collision2->GetLink();
          break;
        } else if (contact->collision2->GetScopedName() == linkScopedCollisionName) {
          graspTargetLinkPtr_ = contact->collision1->GetLink();
          break;
        }
      }

      if (graspTargetLinkPtr_) {
        break;
      }
    }

    if (graspTargetLinkPtr_) {
      graspJoint_->Load(palmLink_, graspTargetLinkPtr_, ignition::math::Pose3d());
      graspJoint_->Init();

      isAttachedToObject_ = true;
    }
  }

  return isAttachedToObject_;
}

void SimGripper::releaseGrasp() {
  if (getIsAttachedToObject()) {
    isAttachedToObject_ = false;
    graspJoint_->Detach();
  }
}

} /* namespace gazebo */
