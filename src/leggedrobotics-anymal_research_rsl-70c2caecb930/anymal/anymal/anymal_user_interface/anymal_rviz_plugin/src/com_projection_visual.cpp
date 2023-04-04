/*
 * com_projection_visual.cpp
 *
 *  Created on: Feb 7, 2018
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/com_projection_visual.hpp"

namespace anymal_rviz_plugin {

ComProjectionVisual::ComProjectionVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : enabled_(true),
      column_enabled_(true),
      color_(Ogre::ColourValue::White),
      com_position_(Ogre::Vector3::ZERO),
      floor_(0.0),
      width_(0.1),
      com_line_(rviz::Shape::Cylinder, scene_manager, parent_node),
      com_point_(rviz::Shape::Sphere, scene_manager, parent_node) {
  com_line_.setOrientation(Ogre::Quaternion(Ogre::Radian(M_PI / 2.0), Ogre::Vector3::UNIT_X));

  com_line_.setScale(Ogre::Vector3::ZERO);
  com_point_.setScale(Ogre::Vector3::ZERO);
}

ComProjectionVisual::~ComProjectionVisual() {}

void ComProjectionVisual::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (enabled_) {
    drawVisual();
  } else {
    com_line_.setScale(Ogre::Vector3::ZERO);
    com_point_.setScale(Ogre::Vector3::ZERO);
  }
}

void ComProjectionVisual::setColor(const Ogre::ColourValue& color) {
  color_ = color;
  com_line_.setColor(color_);
  com_point_.setColor(color_);
}

void ComProjectionVisual::setComPosition(const Ogre::Vector3& com_position) {
  com_position_ = com_position;
  com_line_.setPosition(com_position_);
  drawVisual();
}

void ComProjectionVisual::setEnableColumn(bool enabled) {
  column_enabled_ = enabled;
  if (column_enabled_) {
    drawVisual();
  } else {
    com_line_.setScale(Ogre::Vector3::ZERO);
  }
}

void ComProjectionVisual::setFloor(const Ogre::Real& floor) {
  floor_ = floor;
  drawVisual();
}

void ComProjectionVisual::setWidth(float width) {
  width_ = width;
  drawVisual();
}

void ComProjectionVisual::drawVisual() {
  if (enabled_) {
    if (!std::isfinite(floor_)) {
      com_line_.setScale(Ogre::Vector3::ZERO);
      com_point_.setScale(Ogre::Vector3::ZERO);
    } else {
      if (column_enabled_) {
        com_line_.setScale(Ogre::Vector3(width_, com_position_.z - floor_, width_));
        Ogre::Vector3 cylinder_position(com_position_.x, com_position_.y, (com_position_.z + floor_) / 2.0);
        com_line_.setPosition(cylinder_position);
      }
      com_point_.setScale(Ogre::Vector3(width_ * 4.0));
      Ogre::Vector3 point_position(com_position_.x, com_position_.y, floor_);
      com_point_.setPosition(point_position);
    }
  }
}

}  // namespace anymal_rviz_plugin
