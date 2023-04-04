/*
 * contact_visual.cpp
 *
 *  Created on: Jan 21, 2018
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/contact_visual.hpp"

namespace anymal_rviz_plugin {

ContactVisual::ContactVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager),
      visual_node_(parent_node->createChildSceneNode()),
      position_(Ogre::Vector3::ZERO),
      orientation_(Ogre::Quaternion::IDENTITY),
      normal_(Ogre::Vector3::ZERO),
      force_(Ogre::Vector3::ZERO),
      enabled_(false),
      plane_enabled_(true),
      force_enabled_(true),
      plane_color_(1.0, 1.0, 1.0, 1.0),
      force_color_(1.0, 1.0, 1.0, 1.0),
      plane_size_(0.5),
      force_scale_(1.0),
      arrow_width_(0.05) {
  // Run all the settings options.
  // Technically not neccessary, but has the nice feature that just constructing this object sets all the settings so
  // that the objects are visible.
  allocateWrench();
  setEnabled(enabled_);
}

ContactVisual::~ContactVisual() {
  scene_manager_->destroySceneNode(visual_node_);
}

void ContactVisual::setTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  position_ = position;
  orientation_ = orientation;
  if (enabled_ && plane_enabled_ && contact_plane_) {
    contact_plane_->setPosition(position);
  }
  if (enabled_ && force_enabled_ && wrench_) {
    wrench_->setPosition(position);
    wrench_->setOrientation(orientation);
  }
}

void ContactVisual::setNormal(const Ogre::Vector3& normal) {
  normal_ = normal;
  if (contact_plane_) {
    Ogre::Quaternion normal_orientation = Ogre::Vector3(0.0, 1.0, 0.0).getRotationTo(normal_);
    contact_plane_->setOrientation(normal_orientation * orientation_);
  }
}

void ContactVisual::setWrench(const Ogre::Vector3& force) {
  force_ = force;
  if (enabled_ && force_enabled_ && wrench_) {
    float norm = force_.length();
    if (norm > arrow_width_ * 2) {
      wrench_->setColor(force_color_);
      wrench_->setDirection(force_);
      wrench_->set(norm * force_scale_ - arrow_width_, arrow_width_, arrow_width_ * 1.5, arrow_width_ * 1.5);
    }
  }
}

void ContactVisual::setPlaneColor(const Ogre::ColourValue& plane_color) {
  plane_color_ = plane_color;
  if (contact_plane_) {
    contact_plane_->setColor(plane_color_);
  }
}

void ContactVisual::setForceColor(const Ogre::ColourValue& force_color) {
  force_color_ = force_color;
  if (wrench_) {
    wrench_->setColor(force_color_);
  }
}

void ContactVisual::setPlaneSize(float side_length) {
  plane_size_ = side_length;
  if (contact_plane_) {
    contact_plane_->setScale(Ogre::Vector3(plane_size_, plane_thickness_, plane_size_));
  }
}

void ContactVisual::setArrowWidth(float width) {
  arrow_width_ = width;
  if (wrench_) {
    setWrench(force_);
  }
}

void ContactVisual::setForceScale(float force_scale) {
  force_scale_ = force_scale;
  if (wrench_) {
    setWrench(force_);
  }
}

void ContactVisual::setEnabled(bool enabled) {
  enabled_ = enabled;

  setPlaneVisible(plane_enabled_);
  setForceVisible(force_enabled_);
}

void ContactVisual::setPlaneVisible(bool plane_visible) {
  plane_enabled_ = plane_visible;

  if (enabled_ && plane_enabled_) {
    if (!contact_plane_) {
      allocatePlane();
    }
  } else {
    if (contact_plane_) {
      contact_plane_.reset();
    }
  }
}

void ContactVisual::setForceVisible(bool force_visible) {
  force_enabled_ = force_visible;

  if (enabled_ && force_visible) {
    if (!wrench_) {
      allocateWrench();
    }
  } else {
    if (wrench_) {
      wrench_.reset();
    }
  }

  setWrench(force_);
}

void ContactVisual::allocatePlane() {
  if (!contact_plane_) {
    contact_plane_ = std::make_unique<rviz::Shape>(rviz::Shape::Cylinder, scene_manager_, visual_node_);
    setPlaneColor(plane_color_);
    setPlaneSize(plane_size_);
    setNormal(normal_);
    setTransform(position_, orientation_);
  }
}

void ContactVisual::allocateWrench() {
  if (!wrench_) {
    wrench_ = std::make_unique<rviz::Arrow>(scene_manager_, visual_node_);
    setForceColor(force_color_);
    setForceScale(force_scale_);
    setForceScale(force_scale_);
    setArrowWidth(arrow_width_);
    setTransform(position_, orientation_);

    setWrench(force_);
  }
}

}  // namespace anymal_rviz_plugin
