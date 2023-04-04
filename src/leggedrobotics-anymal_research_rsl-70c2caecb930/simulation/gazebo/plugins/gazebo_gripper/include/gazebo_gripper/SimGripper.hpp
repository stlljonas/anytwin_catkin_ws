/*
 * SimGripper.hpp
 *
 *  Created on: Dec 28, 2018
 *      Author: perry
 */

#pragma once

// gazebo
#include <gazebo/physics/physics.hh>

namespace gazebo {

/**
 * A simple class for approximating a gripper in gazebo.
 *   Given a Collision Link Name (from the URDF), this checks for collisions. If it is in collision,
 *   it creates a fixed joint in gazebo between the Palm Link (from gazebo, NOT neccessarily the URDF)
 *   and the object in collision.
 * This class is not a standalone plugin; instead, use it in a plugin designed for your gripper.
 */
class SimGripper {
 public:
  /**
   * Constructor
   * @param robotName: the name of the robot (or greater model); only used for info messages
   * @param     model: ptr to the gazebo model this operates as a part of
   */
  SimGripper(const std::string& robotName, physics::ModelPtr& model);
  /**
   * Constructor
   * @param         robotName: the name of the robot (or greater model); only used for info messages
   * @param             model: ptr to the gazebo model this operates as a part of
   * @param collisionLinkName: name of the link that collision should be checked with
   * @param      palmLinkName: the link that serves as the parent for the grasp joint (ie the 'grasped' object becomes a child of this link)
   */
  SimGripper(const std::string& robotName, physics::ModelPtr& model, const std::string& collisionLinkName, const std::string& palmLinkName);

  /**
   * Destructor.
   */
  virtual ~SimGripper();

  /**
   * Sets the Collision Link.
   * @param collisionLinkName: name of the link that collision should be checked with
   */
  void setCollisionLinkName(const std::string& collisionLinkName);
  /**
   * Gets the Collision Link.
   */
  const std::string& getCollisionLinkName() const { return collisionLinkName_; }

  /**
   * Sets the Palm Link.
   * @param palmLinkName: the link that serves as the parent for the grasp joint (ie the 'grasped' object becomes a child of this link)
   */
  void setPalmLinkName(const std::string& palmLinkName);
  /**
   * Gets the Palm Link Name.
   */
  std::string getPalmLinkName() const;
  /**
   * Gets a ptr to the Palm Link.
   */
  const physics::LinkPtr& getPalmLink() const { return palmLink_; }

  /**
   * Returns whether the gripper is attached to an object (IE is the graspJoint active)
   */
  bool getIsAttachedToObject() const { return isAttachedToObject_; }

  /**
   * Checks if the gripper is in colllision with an object, and then attaches the object if it is.
   */
  bool attachGrasp();

  /**
   *Releases the grasped object if it exists (detaches the joint).
   */
  void releaseGrasp();

 private:
  // The name of the robot.
  const std::string& robotName_;
  // The gazebo physics model of the robot.
  const physics::ModelPtr model_;

  // The name of the collision link.
  std::string collisionLinkName_;
  // The palm link (in the gazebo model).
  physics::LinkPtr palmLink_;

  // A list of names of the collision objects beloning to the collision link.
  std::vector<std::string> linkScopedCollisionNames_;

  // The fixed joint used to simulate a successful grasp.
  physics::JointPtr graspJoint_;

  // Flag for if the gripper is attached to an object (IE is the graspJoint active)
  bool isAttachedToObject_;
};

} /* namespace gazebo */
