/*
 * velocity_visual.cpp
 *
 *  Created on: Mar 2, 2018
 *      Author: Perry Franklin
 */

#include <anymal_rviz_plugin/velocity_visual.hpp>

namespace anymal_rviz_plugin {

VelocityVisual::VelocityVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : visual_node_(parent_node->createChildSceneNode()),
      enabled_(true),
      linear_enabled_(true),
      angular_enabled_(true),
      position_(Ogre::Vector3::ZERO),
      linear_velocity_(Ogre::Vector3::ZERO),
      angular_velocity_(Ogre::Vector3::ZERO),
      width_(0.01),
      linear_scale_(1.0),
      angular_scale_(0.5),
      linear_visual_(scene_manager, visual_node_),
      angular_visual_(scene_manager, visual_node_),
      angular_circle_visual_(scene_manager, visual_node_),
      angular_circle_end_visual_(scene_manager, visual_node_) {}

VelocityVisual::~VelocityVisual() {}

void VelocityVisual::setPosition(const Ogre::Vector3& position) {
  position_ = position;
}

void VelocityVisual::setTwist(const Ogre::Vector3& linear, const Ogre::Vector3& angular) {
  linear_velocity_ = linear;
  angular_velocity_ = angular;
  drawVisual();
}

void VelocityVisual::setWidth(double width) {
  width_ = width;
}

void VelocityVisual::setLinearColor(const Ogre::ColourValue& linear_color) {
  linear_visual_.setColor(linear_color);
}

void VelocityVisual::setAngularColor(const Ogre::ColourValue& angular_color) {
  angular_visual_.setColor(angular_color);
  angular_circle_visual_.setColor(angular_color.r, angular_color.g, angular_color.b, angular_color.a);
  angular_circle_end_visual_.setColor(angular_color);
}

void VelocityVisual::setLinearScale(double linear_scale) {
  linear_scale_ = linear_scale;
  drawVisual();
}

void VelocityVisual::setAngularScale(double angular_scale) {
  angular_scale_ = angular_scale;
  drawVisual();
}

void VelocityVisual::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (enabled_) {
    drawVisual();
  } else {
    clearLinearVisual();
    clearAngularVisual();
  }
}

void VelocityVisual::setLinearEnabled(bool linear_enabled) {
  linear_enabled_ = linear_enabled;
  if (linear_enabled_) {
    drawVisual();
  } else {
    clearLinearVisual();
  }
}

void VelocityVisual::setAngularEnabled(bool angular_enabled) {
  angular_enabled_ = angular_enabled;
  if (angular_enabled_) {
    drawVisual();
  } else {
    clearAngularVisual();
  }
}

void VelocityVisual::drawVisual() {
  if (enabled_) {
    if (linear_enabled_) {
      if (linear_velocity_.length() > width_) {
        linear_visual_.set(linear_velocity_.length() * linear_scale_, width_, width_ * 2.0, width_ * 2.5);
        linear_visual_.setPosition(position_);
        linear_visual_.setDirection(linear_velocity_);
      } else {
        clearLinearVisual();
      }
    }
    if (angular_enabled_) {
      if (angular_velocity_.length() > width_) {
        double angular_magnitude = angular_velocity_.length();
        angular_visual_.set(angular_velocity_.length() * angular_scale_, width_, width_ * 2.0, width_ * 2.5);
        angular_visual_.setPosition(position_);
        angular_visual_.setDirection(angular_velocity_);
        createCircle(position_ + (angular_velocity_ * angular_scale_ * 0.5), angular_velocity_, width_ * 0.5,
                     angular_magnitude * angular_scale_ * 0.25);
      } else {
        clearAngularVisual();
      }
    }
  }
}

void VelocityVisual::createCircle(const Ogre::Vector3& center, const Ogre::Vector3& axis, double width, double diameter) {
  angular_circle_visual_.setPosition(center);
  Ogre::Quaternion orientation = Ogre::Vector3::UNIT_Z.getRotationTo(axis, Ogre::Vector3::UNIT_X);

  angular_circle_visual_.setOrientation(orientation);

  angular_circle_visual_.setLineWidth(width);

  angular_circle_visual_.clear();
  const static double angle_delta = 0.15;
  const static double angle_start = 0.8;
  const static double angle_end = 2.0 * M_PI;

  for (double angle = angle_start; angle < angle_end; angle += angle_delta) {
    Ogre::Vector3 point(diameter * cos(angle), diameter * sin(angle), 0.0);
    angular_circle_visual_.addPoint(point);
  }
  Ogre::Vector3 final_point(diameter * cos(angle_end), diameter * sin(angle_end), 0.0);
  angular_circle_visual_.addPoint(final_point);

  angular_circle_end_visual_.setPosition(orientation * final_point + center);
  angular_circle_end_visual_.set(0.0, width, width * 2, width * 2);
  angular_circle_end_visual_.setDirection(orientation * Ogre::Quaternion(Ogre::Radian(M_PI_2), Ogre::Vector3::UNIT_Z) * final_point);
}

void VelocityVisual::clearLinearVisual() {
  linear_visual_.set(0.0, 0.0, 0.0, 0.0);
}

void VelocityVisual::clearAngularVisual() {
  angular_visual_.set(0.0, 0.0, 0.0, 0.0);
  angular_circle_visual_.clear();
  angular_circle_end_visual_.set(0.0, 0.0, 0.0, 0.0);
}

}  // namespace anymal_rviz_plugin
