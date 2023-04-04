/*
 * joint_torque.cpp
 *
 *  Created on: Dec 28, 2017
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/joint_torque.hpp"

#include "anymal_rviz_plugin/joint_torque_tree.hpp"

namespace anymal_rviz_plugin {

JointTorque::JointTorque(const std::string& name, rviz::Property* parent_property, Ogre::SceneManager* manager, Ogre::SceneNode* scene_node)
    : name_(name), child_link_name_(""), visual_(manager, scene_node) {
  joint_property_ = new rviz::BoolProperty(name_.c_str(), true, "Enable the visual for this Joint Torque", parent_property,
                                           SLOT(enableJointTorque()), this);

  offset_property_ = new rviz::FloatProperty("Offset", 0.0, "Offset of the Joint Torque visual (along the joint axis)", joint_property_,
                                             SLOT(changedOffset()), this);

  torque_max_property_ =
      new rviz::FloatProperty("Torque Maximum", 40.0, "Maximum torque of the Joint, which corresponds to a full visual circle (2PI)",
                              joint_property_, SLOT(changedTorqueMax()), this);
  torque_max_property_->setMin(0.0);

  joint_property_->setDisableChildrenIfFalse(true);

  changedOffset();
  changedTorqueMax();
}

JointTorque::~JointTorque() {
  delete offset_property_;
  delete torque_max_property_;

  delete joint_property_;
}

bool JointTorque::getEnabled() const {
  return joint_property_->getValue().toBool();
}

void JointTorque::update(const AnymalLinkUpdater& updater_) {
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  updater_.getLinkTransforms(child_link_name_, position, orientation, position, orientation);

  visual_.setTransform(position, orientation);

  const anymal_model::AnymalModel& quad_model = updater_.getModel();

  const int index = anymal_description::AnymalTopology::jointKeys.atName(name_).getId();

  const anymal_model::JointTorques& torques = quad_model.getJointTorques();

  visual_.setMagnitude(torques(index));
  visual_.drawVisual();
}

void JointTorque::enableJointTorque() {
  visual_.setEnabled(joint_property_->getValue().toBool());
}

void JointTorque::changedOffset() {
  visual_.setOffset(offset_property_->getFloat());
}

void JointTorque::changedTorqueMax() {
  visual_.setMagnitudeMax(torque_max_property_->getFloat());
}

void JointTorque::setColorMode(JointTorqueVisual::JointTorqueColorModeEnum color_mode) {
  visual_.setColorMode(color_mode);
}

void JointTorque::setThreshold(double threshold) {
  visual_.setThreshold(threshold);
}

void JointTorque::setColor(const Ogre::ColourValue& color) {
  visual_.setColor(color);
}

void JointTorque::setScale(double scale) {
  visual_.setScale(scale);
}

void JointTorque::setUrdfProperties(const urdf::JointConstSharedPtr& joint) {
  if (name_ != joint->name) {
    ROS_WARN_STREAM("joint_torque.cpp: somehow you've tried to initialize the joint " << name_ << " with the URDF joint " << joint->name);
  }

  child_link_name_ = joint->child_link_name;

  visual_.setAxis(Ogre::Vector3(joint->axis.x, joint->axis.y, joint->axis.z));
}

}  // namespace anymal_rviz_plugin
