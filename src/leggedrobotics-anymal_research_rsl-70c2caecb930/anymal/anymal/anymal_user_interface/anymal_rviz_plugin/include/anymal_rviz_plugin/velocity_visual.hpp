/*
 * velocity_visual.hpp
 *
 *  Created on: Mar 2, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

namespace anymal_rviz_plugin {

class VelocityVisual {
 public:
  VelocityVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~VelocityVisual();

  void setPosition(const Ogre::Vector3& position);
  void setTwist(const Ogre::Vector3& linear, const Ogre::Vector3& angular);

  void setWidth(double width);

  void setLinearColor(const Ogre::ColourValue& linear_color);
  void setAngularColor(const Ogre::ColourValue& angular_color);

  void setLinearScale(double linear_scale);
  void setAngularScale(double angular_scale);

  void setEnabled(bool enabled);
  void setLinearEnabled(bool linear_enabled);
  void setAngularEnabled(bool angular_enabled);

 private:
  void drawVisual();
  void createCircle(const Ogre::Vector3& center, const Ogre::Vector3& axis, double width, double diameter);
  void clearLinearVisual();
  void clearAngularVisual();

  Ogre::SceneNode* visual_node_;

  bool enabled_;
  bool linear_enabled_;
  bool angular_enabled_;

  Ogre::Vector3 position_;
  Ogre::Vector3 linear_velocity_;
  Ogre::Vector3 angular_velocity_;

  double width_;
  double linear_scale_;
  double angular_scale_;

  rviz::Arrow linear_visual_;
  rviz::Arrow angular_visual_;
  rviz::BillboardLine angular_circle_visual_;
  rviz::Arrow angular_circle_end_visual_;
};

}  // namespace anymal_rviz_plugin
