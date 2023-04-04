/*
 * joint_torque_tree.hpp
 *
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/joint_torque_tree.hpp"

#include <memory>

#include <ros/assert.h>
#include <ros/console.h>

using namespace rviz;

namespace anymal_rviz_plugin {

JointTorqueColorOptionProperty::JointTorqueColorOptionProperty(const QString& name,
                                                               JointTorqueVisual::JointTorqueColorModeEnum default_mode,
                                                               const QString& description, rviz::Property* parent, const char* changed_slot,
                                                               QObject* receiver)
    : EnumProperty(name, JointTorqueColorModeMap.at(default_mode), description, parent, changed_slot, receiver) {
  for (const std::pair<int, QString>& num_option_pair : JointTorqueColorModeMap) {
    this->addOption(num_option_pair.second, num_option_pair.first);
  }
}

JointTorqueTree::JointTorqueTree(Ogre::SceneNode* root_node, DisplayContext* context, const std::string& name, Property* parent_property)
    : scene_manager_(context->getSceneManager()), visible_(true), context_(context), robot_loaded_(false), name_(name) {
  root_visual_node_ = root_node->createChildSceneNode();

  joint_torques_property_ = new BoolProperty("Joint Torques", true, "", parent_property, SLOT(enableAllLinks()), this);

  color_mode_property_ = new JointTorqueColorOptionProperty("Color Mode", JointTorqueVisual::SOLID_COLOR,
                                                            "How the Color of the Joint Torque visual is calculated",
                                                            joint_torques_property_, SLOT(changedColorMode()), this);

  threshold_property_ = new FloatProperty("Threshold", 0.75, "Threshold for the Joint Torque visual to turn red", color_mode_property_,
                                          SLOT(changedThreshold()), this);
  threshold_property_->setMin(0.0);
  threshold_property_->setMax(1.0);

  color_property_ = new ColorProperty("Color", QColor(240, 240, 240), "Color of the Joint Torque visuals", joint_torques_property_,
                                      SLOT(changedColorAlpha()), this);

  alpha_property_ =
      new FloatProperty("Alpha", 1.0, "Transparency of the Joint Torque visuals", joint_torques_property_, SLOT(changedColorAlpha()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  scale_property_ = new FloatProperty("Size", 0.1, "Size of the Joint Torque visuals", joint_torques_property_, SLOT(changedScale()), this);
  scale_property_->setMin(0.0);

  for (auto element : anymal_description::AnymalTopology::jointKeys) {
    joints_torques_[element.getName()] =
        std::make_unique<JointTorque>(element.getName(), joint_torques_property_, scene_manager_, root_visual_node_);
  }

  joint_torques_property_->setDisableChildrenIfFalse(true);

  changedColorMode();
  changedThreshold();
  changedColorAlpha();
  changedScale();
}

JointTorqueTree::~JointTorqueTree() {
  joints_torques_.clear();
  root_visual_node_->removeAndDestroyAllChildren();

  delete threshold_property_;
  delete color_mode_property_;
  delete color_property_;
  delete alpha_property_;
  delete scale_property_;
  delete joint_torques_property_;
}

void JointTorqueTree::load_urdf(const urdf::ModelInterface& urdf) {
  joint_torques_property_->hide();  // hide until loaded
  robot_loaded_ = false;

  {
    typedef std::map<std::string, urdf::JointSharedPtr> M_NameToUrdfJoint;
    M_NameToUrdfJoint::const_iterator joint_it = urdf.joints_.begin();
    M_NameToUrdfJoint::const_iterator joint_end = urdf.joints_.end();
    for (; joint_it != joint_end; ++joint_it) {
      const urdf::JointConstSharedPtr& urdf_joint = joint_it->second;

      if (urdf_joint->type != urdf::Joint::REVOLUTE) {
        continue;
      }

      joints_torques_[urdf_joint->name]->setUrdfProperties(urdf_joint);
    }
  }

  // robot is now loaded
  robot_loaded_ = true;
  joint_torques_property_->show();

  joint_torques_property_->setName("Joint Torques");
  joint_torques_property_->setDescription("All joints in the robot in alphabetic order.");

  // at startup the link tree is collapsed since it is large and not often needed.
  joint_torques_property_->collapse();
}

void JointTorqueTree::enableAllLinks() {
  bool enable = joint_torques_property_->getBool();

  M_NameToJointTorque::iterator joint_torque_it = joints_torques_.begin();
  M_NameToJointTorque::iterator joint_torque_end = joints_torques_.end();
  for (; joint_torque_it != joint_torque_end; ++joint_torque_it) {
    joint_torque_it->second->getJointProperty()->setValue(enable);
  }
}

void JointTorqueTree::changedColorMode() {
  M_NameToJointTorque::iterator joint_torque_it = joints_torques_.begin();
  M_NameToJointTorque::iterator joint_torque_end = joints_torques_.end();
  for (; joint_torque_it != joint_torque_end; ++joint_torque_it) {
    joint_torque_it->second->setColorMode((JointTorqueVisual::JointTorqueColorModeEnum)color_mode_property_->getOptionInt());
  }
  if ((JointTorqueVisual::JointTorqueColorModeEnum)color_mode_property_->getOptionInt() ==
      JointTorqueVisual::JointTorqueColorModeEnum::COLOR_MAGNITUDE_THRESHOLD) {
    threshold_property_->setHidden(false);
    color_mode_property_->expand();
  } else {
    threshold_property_->hide();
  }
}

void JointTorqueTree::changedThreshold() {
  M_NameToJointTorque::iterator joint_torque_it = joints_torques_.begin();
  M_NameToJointTorque::iterator joint_torque_end = joints_torques_.end();
  for (; joint_torque_it != joint_torque_end; ++joint_torque_it) {
    joint_torque_it->second->setThreshold(threshold_property_->getFloat());
  }
}

void JointTorqueTree::changedColorAlpha() {
  Ogre::ColourValue color = rviz::qtToOgre(color_property_->getColor());
  color.a = alpha_property_->getFloat();

  M_NameToJointTorque::iterator joint_torque_it = joints_torques_.begin();
  M_NameToJointTorque::iterator joint_torque_end = joints_torques_.end();
  for (; joint_torque_it != joint_torque_end; ++joint_torque_it) {
    joint_torque_it->second->setColor(color);
  }
}

void JointTorqueTree::changedScale() {
  M_NameToJointTorque::iterator joint_torque_it = joints_torques_.begin();
  M_NameToJointTorque::iterator joint_torque_end = joints_torques_.end();
  for (; joint_torque_it != joint_torque_end; ++joint_torque_it) {
    joint_torque_it->second->setScale(scale_property_->getFloat());
  }
}

void JointTorqueTree::update(const AnymalLinkUpdater& updater) {
  for (auto& pair : joints_torques_) {
    std::unique_ptr<JointTorque>& joint_torque = pair.second;
    joint_torque->update(updater);
  }
}

}  // namespace anymal_rviz_plugin
