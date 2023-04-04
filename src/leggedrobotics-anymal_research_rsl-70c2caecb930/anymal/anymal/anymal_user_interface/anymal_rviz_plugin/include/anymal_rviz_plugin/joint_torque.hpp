/*
 * joint_torque.hpp
 *
 *  Created on: Dec 28, 2017
 *      Author: Perry Franklin
 */
#pragma once

#include <map>
#include <string>

#include <QObject>

#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <urdf/model.h>
#include <urdf_model/pose.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/forwards.h>

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/joint_torque_visual.hpp"

namespace anymal_rviz_plugin {

class JointTorque : public QObject {
  Q_OBJECT
 public:
  JointTorque(const std::string& name, rviz::Property* parent_property, Ogre::SceneManager* manager, Ogre::SceneNode* scene_node);
  virtual ~JointTorque();

  void setColorMode(JointTorqueVisual::JointTorqueColorModeEnum color_mode);
  void setThreshold(double threshold);
  void setColor(const Ogre::ColourValue& color);
  void setScale(double scale);

  const std::string& getName() const { return name_; }
  const rviz::Property* getJointProperty() const { return joint_property_; }
  rviz::Property* getJointProperty() { return joint_property_; }

  void update(const AnymalLinkUpdater& updater_);

  void setUrdfProperties(const urdf::JointConstSharedPtr& joint);

 private Q_SLOTS:
  void enableJointTorque();
  void changedOffset();
  void changedTorqueMax();

 private:
  bool getEnabled() const;

 protected:
  std::string name_;
  std::string child_link_name_;

  rviz::BoolProperty* joint_property_;
  rviz::FloatProperty* offset_property_;
  rviz::FloatProperty* torque_max_property_;

 private:
  JointTorqueVisual visual_;
};

}  // namespace anymal_rviz_plugin
