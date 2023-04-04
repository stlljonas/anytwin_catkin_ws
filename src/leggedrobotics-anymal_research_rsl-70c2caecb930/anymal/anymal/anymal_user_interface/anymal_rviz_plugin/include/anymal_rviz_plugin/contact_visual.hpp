/*
 * contact_visual.hpp
 *
 *  Created on: Jan 1, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <memory>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>

namespace anymal_rviz_plugin {

class ContactVisual {
 public:
  ContactVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~ContactVisual();

  // We don't use the orientation
  void setTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation = Ogre::Quaternion(1.0, 0.0, 0.0, 0.0));
  void setNormal(const Ogre::Vector3& normal);

  void setWrench(const Ogre::Vector3& force);

  void setEnabled(bool enabled);

  void setPlaneColor(const Ogre::ColourValue& plane_color);
  void setForceColor(const Ogre::ColourValue& force_color);

  void setPlaneSize(float side_length);
  void setForceScale(float force_scale);
  void setArrowWidth(float width);

  void setPlaneVisible(bool plane_visible);
  void setForceVisible(bool force_visible);

 private:
  void allocatePlane();
  void allocateWrench();

  Ogre::SceneManager* const scene_manager_;
  Ogre::SceneNode* const visual_node_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  Ogre::Vector3 normal_;
  Ogre::Vector3 force_;

  std::unique_ptr<rviz::Shape> contact_plane_;

  std::unique_ptr<rviz::Arrow> wrench_;

  const double plane_thickness_ = 0.002;

  bool enabled_;
  bool plane_enabled_;
  bool force_enabled_;
  Ogre::ColourValue plane_color_;
  Ogre::ColourValue force_color_;
  float plane_size_;
  float force_scale_;
  float arrow_width_;
};

}  // namespace anymal_rviz_plugin
