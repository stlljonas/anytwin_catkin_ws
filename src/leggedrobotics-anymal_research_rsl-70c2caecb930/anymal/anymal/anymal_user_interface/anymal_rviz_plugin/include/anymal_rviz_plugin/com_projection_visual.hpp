/*
 * com_projection_visual.hpp
 *
 *  Created on: Feb 7, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <OgreColourValue.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz/ogre_helpers/shape.h>

namespace anymal_rviz_plugin {

class ComProjectionVisual {
 public:
  ComProjectionVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~ComProjectionVisual();

  void setEnabled(bool enabled);
  void setColor(const Ogre::ColourValue& color);
  void setComPosition(const Ogre::Vector3& com_position);
  void setEnableColumn(bool enabled);

  // If floor is NaN, then the arrow is not displayed
  void setFloor(const Ogre::Real& floor);
  void setWidth(float width);

 private:
  // Attempts to create the correct arrow. If floor is NaN, hides the arrow
  void drawVisual();

  bool enabled_;
  bool column_enabled_;
  Ogre::ColourValue color_;
  Ogre::Vector3 com_position_;
  Ogre::Real floor_;
  float width_;

  rviz::Shape com_line_;
  rviz::Shape com_point_;
};

}  // namespace anymal_rviz_plugin
