/*
 * velocity.cpp
 *
 *  Created on: Jan 8, 2018
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/velocity.hpp"

namespace anymal_rviz_plugin {

Velocity::Velocity(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property)
    : linear_enabled_(true),
      linear_velocity_(Ogre::Vector3::ZERO),
      angular_enabled_(true),
      angular_velocity_(Ogre::Vector3::ZERO),
      offset_(0.0),
      visual_position_(Ogre::Vector3::ZERO) {
  velocity_property_ = new rviz::BoolProperty("Robot Velocity", true, "", parent_property, SLOT(enableVelocityProperty()), this);

  alpha_property_ =
      new rviz::FloatProperty("Alpha", 0.6, "Transparency of the wrench visuals", velocity_property_, SLOT(changeAlpha()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  offset_property_ =
      new rviz::FloatProperty("Offset", 0.4, "Z-offset of the visuals from the body", velocity_property_, SLOT(changeOffset()), this);

  linear_show_property_ =
      new rviz::BoolProperty("Show Linear", true, "Enable Linear Velocity visual", velocity_property_, SLOT(enableLinearShow()), this);

  linear_color_property_ = new rviz::ColorProperty("Linear Color", QColor(0, 0, 204), "Color of the Linear visuals", velocity_property_,
                                                   SLOT(changeLinearColor()), this);

  linear_scale_property_ =
      new rviz::FloatProperty("Linear Scale", 1.0, "Scale of the visuals", velocity_property_, SLOT(changeLinearScale()), this);
  linear_scale_property_->setMin(0.0);

  angular_show_property_ =
      new rviz::BoolProperty("Show Angular", true, "Enable Angular Velocity visual", velocity_property_, SLOT(enableAngularShow()), this);

  angular_color_property_ = new rviz::ColorProperty("Angular Color", QColor(0, 204, 0), "Color of the angular visuals", velocity_property_,
                                                    SLOT(changeAngularColor()), this);

  angular_scale_property_ =
      new rviz::FloatProperty("Angular Scale", 0.5, "Scale of the visuals", velocity_property_, SLOT(changeAngularScale()), this);
  angular_scale_property_->setMin(0.0);

  visual_ = std::make_unique<VelocityVisual>(context->getSceneManager(), root_node);

  visual_->setWidth(0.01);

  velocity_property_->setDisableChildrenIfFalse(true);

  enableLinearShow();
  enableAngularShow();
  changeLinearColor();
  changeAngularColor();
  changeAlpha();
  changeLinearScale();
  changeAngularScale();
  changeOffset();
}

Velocity::~Velocity() {
  delete alpha_property_;
  delete offset_property_;
  delete linear_show_property_;
  delete linear_color_property_;
  delete linear_scale_property_;
  delete angular_show_property_;
  delete angular_color_property_;
  delete angular_scale_property_;

  delete velocity_property_;
}

void Velocity::enableLinearShow() {
  linear_enabled_ = linear_show_property_->getBool();
  setVisualVelocity();
}

void Velocity::changeAlpha() {
  changeLinearColor();
  changeAngularColor();
}

void Velocity::changeLinearScale() {
  visual_->setLinearScale(linear_scale_property_->getFloat());
  setVisualVelocity();
}

void Velocity::changeAngularScale() {
  visual_->setAngularScale(angular_scale_property_->getFloat());
  setVisualVelocity();
}

void Velocity::changeLinearColor() {
  Ogre::ColourValue color = rviz::qtToOgre(linear_color_property_->getColor());
  color.a = alpha_property_->getFloat();
  visual_->setLinearColor(color);
}

void Velocity::enableAngularShow() {
  angular_enabled_ = angular_show_property_->getBool();
  setVisualVelocity();
}

void Velocity::changeAngularColor() {
  Ogre::ColourValue color = rviz::qtToOgre(angular_color_property_->getColor());
  color.a = alpha_property_->getFloat();
  visual_->setAngularColor(color);
}

void Velocity::changeOffset() {
  double old_offset_ = offset_;
  offset_ = offset_property_->getFloat();
  visual_position_.z = visual_position_.z - old_offset_ + offset_;
  visual_->setPosition(visual_position_);
  visual_->setTwist(linear_velocity_, angular_velocity_);
}

void Velocity::enableVelocityProperty() {
  setVisualVelocity();
}

void Velocity::update(const AnymalLinkUpdater& anymal_link_updater) {
  const anymal_model::AnymalModel& model = anymal_link_updater.getModel();

  Eigen::Vector3d position = model.getPositionWorldToBody(anymal_description::AnymalTopology::BodyEnum::BASE,
                                                          anymal_description::AnymalTopology::CoordinateFrameEnum::WORLD);
  visual_position_ = Ogre::Vector3(position.x(), position.y(), position.z() + offset_);

  visual_->setPosition(visual_position_);

  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
  // Someone should look at why these coordinate frames make the visualization correct...
  linear_velocity = model.getLinearVelocityWorldToBody(anymal_description::AnymalTopology::BodyEnum::BASE,
                                                       anymal_description::AnymalTopology::CoordinateFrameEnum::BASE);
  angular_velocity = model.getAngularVelocityWorldToBody(anymal_description::AnymalTopology::BodyEnum::BASE,
                                                         anymal_description::AnymalTopology::CoordinateFrameEnum::WORLD);

  linear_velocity_.x = linear_velocity.x();
  linear_velocity_.y = linear_velocity.y();
  linear_velocity_.z = linear_velocity.z();
  angular_velocity_.x = angular_velocity.x();
  angular_velocity_.y = angular_velocity.y();
  angular_velocity_.z = angular_velocity.z();

  setVisualVelocity();
}

void Velocity::setVisualVelocity() {
  Ogre::Vector3 final_linear_velocity_ = Ogre::Vector3::ZERO;
  Ogre::Vector3 final_angular_velocity_ = Ogre::Vector3::ZERO;

  if (linear_enabled_ && velocity_property_->getBool()) {
    final_linear_velocity_ = linear_velocity_;
  }

  if (angular_enabled_ && velocity_property_->getBool()) {
    final_angular_velocity_ = angular_velocity_;
  }

  visual_->setTwist(final_linear_velocity_, final_angular_velocity_);
}

}  // namespace anymal_rviz_plugin
