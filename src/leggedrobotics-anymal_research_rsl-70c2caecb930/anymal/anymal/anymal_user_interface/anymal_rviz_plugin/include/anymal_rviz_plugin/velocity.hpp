/*
 * velocity.hpp
 *
 *  Created on: Jan 8, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <QObject>

#include <OgreVector3.h>

#include <rviz/default_plugin/wrench_visual.h>
#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/velocity_visual.hpp"

namespace anymal_rviz_plugin {

class Velocity : QObject {
  Q_OBJECT
 public:
  Velocity(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property);
  ~Velocity();

  void update(const AnymalLinkUpdater& anymal_link_updater);

 private Q_SLOTS:
  void enableVelocityProperty();
  void enableLinearShow();
  void changeLinearColor();
  void changeAlpha();
  void enableAngularShow();
  void changeAngularColor();
  void changeLinearScale();
  void changeAngularScale();
  void changeOffset();

 private:
  void setVisualVelocity();

  bool linear_enabled_;
  Ogre::Vector3 linear_velocity_;

  bool angular_enabled_;
  Ogre::Vector3 angular_velocity_;

  double offset_;
  Ogre::Vector3 visual_position_;

  rviz::BoolProperty* velocity_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* offset_property_;
  rviz::BoolProperty* linear_show_property_;
  rviz::ColorProperty* linear_color_property_;
  rviz::FloatProperty* linear_scale_property_;
  rviz::BoolProperty* angular_show_property_;
  rviz::ColorProperty* angular_color_property_;
  rviz::FloatProperty* angular_scale_property_;

  std::unique_ptr<VelocityVisual> visual_;
};

}  // namespace anymal_rviz_plugin
