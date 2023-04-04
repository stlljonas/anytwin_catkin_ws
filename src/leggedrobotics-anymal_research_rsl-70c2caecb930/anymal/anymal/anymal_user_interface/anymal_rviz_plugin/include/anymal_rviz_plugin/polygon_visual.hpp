/*
 * polygon_visual.hpp
 *
 *  Created on: Jan 18, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <memory>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/mesh_shape.h>

namespace anymal_rviz_plugin {

class PolygonVisual {
 public:
  PolygonVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~PolygonVisual();

  void setVertices(const std::vector<Ogre::Vector3>& points);

  void drawVisual();

  void setEnabled(bool enabled);

  void setFilled(bool fill);

  void setLineColorAlpha(const Ogre::ColourValue& color);
  void setLineThickness(float thickness);

 private:
  Ogre::SceneManager* const scene_manager_;
  Ogre::SceneNode* const frame_node_;

  bool enabled_;

  Ogre::ColourValue color_;
  float thickness_;

  bool filled_;

  std::vector<Ogre::Vector3> points_;

  std::unique_ptr<rviz::BillboardLine> edges_;
  std::unique_ptr<rviz::MeshShape> fill_mesh_;
};

}  // namespace anymal_rviz_plugin
