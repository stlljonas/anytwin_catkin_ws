/*
 * joint_torque_tree.hpp
 *
 *      Author: Perry Franklin
 */

#pragma once

#include <map>
#include <string>

#include <OgreAny.h>
#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/object.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>

#include <urdf/model.h>  // can be replaced later by urdf_model/types.h

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/joint_torque.hpp"
#include "anymal_rviz_plugin/joint_torque_visual.hpp"

namespace anymal_rviz_plugin {

const std::map<int, QString> JointTorqueColorModeMap = {{JointTorqueVisual::SOLID_COLOR, "Solid Color"},
                                                        {JointTorqueVisual::COLOR_MAGNITUDE, "Magnitude"},
                                                        {JointTorqueVisual::COLOR_MAGNITUDE_RAINBOW, "Magnitude Rainbow"},
                                                        {JointTorqueVisual::COLOR_MAGNITUDE_THRESHOLD, "Magnitude Threshold"}};

// Helper class for creating an EnumProperty with the same enum list every time.
class JointTorqueColorOptionProperty : public rviz::EnumProperty {
 public:
  JointTorqueColorOptionProperty(const QString& name, JointTorqueVisual::JointTorqueColorModeEnum default_mode, const QString& description,
                                 rviz::Property* parent, const char* changed_slot, QObject* receiver);
};

class JointTorqueTree : public QObject {
  Q_OBJECT
 public:
  JointTorqueTree(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property);
  virtual ~JointTorqueTree();

  void load_urdf(const urdf::ModelInterface& urdf);

  void update(const AnymalLinkUpdater& updater);

 private Q_SLOTS:
  void enableAllLinks();
  void changedColorMode();
  void changedThreshold();
  void changedColorAlpha();
  void changedScale();

 protected:
  Ogre::SceneManager* scene_manager_;

  typedef std::map<std::string, std::unique_ptr<JointTorque> > M_NameToJointTorque;
  M_NameToJointTorque joints_torques_;

  Ogre::SceneNode* root_visual_node_;

  bool visible_;

  rviz::DisplayContext* context_;
  rviz::BoolProperty* joint_torques_property_;
  JointTorqueColorOptionProperty* color_mode_property_;
  rviz::FloatProperty* threshold_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* scale_property_;

  bool robot_loaded_;  // true after robot model is loaded.

  std::string name_;
  float alpha_;
};

}  // namespace anymal_rviz_plugin
