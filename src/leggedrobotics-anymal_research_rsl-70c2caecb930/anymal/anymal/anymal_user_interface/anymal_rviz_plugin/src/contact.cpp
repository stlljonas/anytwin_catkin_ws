/*
 * contact.cpp
 *
 *  Created on: Jan 1, 2018
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/contact.hpp"

namespace anymal_rviz_plugin {

ContactOptionProperty::ContactOptionProperty(const QString& name, ContactColorModeEnum default_mode, const QString& description,
                                             rviz::Property* parent, const char* changed_slot, QObject* receiver)
    : EnumProperty(name, ContactColorModeMap.at(default_mode), description, parent, changed_slot, receiver) {
  for (const std::pair<int, QString>& num_option_pair : ContactColorModeMap) {
    this->addOption(num_option_pair.second, num_option_pair.first);
  }
}

Contact::Contact(const std::string& name, rviz::Property* parent_property, Ogre::SceneManager* manager, Ogre::SceneNode* scene_node_)
    : name_(name), valid_draw_(false), visual_(manager, scene_node_) {
  contact_property_ = new rviz::BoolProperty(name.c_str(), true, "", parent_property, SLOT(enableContact()), this);

  color_mode_property_ =
      new ContactOptionProperty("Color Mode", ContactColorModeEnum::CONTACTPLANE, "", contact_property_, SLOT(changeColorMode()), this);

  color_property_ =
      new rviz::ColorProperty("Color", QColor(204, 41, 204), "Color of the contact", contact_property_, SLOT(changeColorAlpha()), this);

  alpha_property_ =
      new rviz::FloatProperty("Alpha", 0.3, "Alpha (transparency) of the contact", contact_property_, SLOT(changeColorAlpha()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  wrench_scale_property_ =
      new rviz::FloatProperty("Wrench Scale", 0.003, "Scale of the force/torque", contact_property_, SLOT(changeWrenchScale()), this);
  wrench_scale_property_->setMin(0.0);

  wrench_width_property_ = new rviz::FloatProperty("Wrench Arrow Width", 0.01, "Width of the force/torque arrow", contact_property_,
                                                   SLOT(changeWrenchWidth()), this);
  wrench_width_property_->setMin(0.0);

  plane_size_property_ = new rviz::FloatProperty("Normal Plane Size", 0.2, "Size of the of the contact normal plane", contact_property_,
                                                 SLOT(changePlaneSize()), this);
  plane_size_property_->setMin(0.0);

  show_wrench_property_ = new rviz::BoolProperty("Show Wrench", true, "Show the wrench (force/torque) visual for the contact",
                                                 contact_property_, SLOT(enableShowWrench()), this);

  show_normal_property_ = new rviz::BoolProperty("Show Normal", true, "Show the normal plane of the contact", contact_property_,
                                                 SLOT(enableShowNormal()), this);

  contact_property_->setDisableChildrenIfFalse(true);

  changeColorMode();
  changeColorAlpha();
  changeWrenchScale();
  changeWrenchWidth();
  changePlaneSize();
  enableShowWrench();
  enableShowNormal();
}

Contact::~Contact() {
  delete color_mode_property_;
  delete color_property_;
  delete alpha_property_;
  delete wrench_scale_property_;
  delete wrench_width_property_;
  delete plane_size_property_;
  delete show_wrench_property_;
  delete show_normal_property_;

  delete contact_property_;
}

void Contact::update(const AnymalLinkUpdater& updater) {
  if (!contact_property_->getBool()) {
    // We are disabled (unchecked)
    return;
  }
  auto contact_id = anymal_description::AnymalTopology::contactKeys.atName(name_).getEnum();

  const anymal_model::AnymalModel& model = updater.getModel();

  const romo::ContactShPtrContainer<anymal_description::ConcreteAnymalDescription>& contact_container = model.getContactContainer();

  const std::shared_ptr<romo::Contact<anymal_description::ConcreteAnymalDescription>>& contact = contact_container.at(contact_id);

  if (contact->getState() == romo::Contact<anymal_description::ConcreteAnymalDescription>::ContactState::OPEN) {
    visual_.setEnabled(false);
    valid_draw_ = false;
    return;
  }

  valid_draw_ = true;
  visual_.setEnabled(true);

  romo::Position position = contact->getPositionWorldToContact(anymal_model::AnymalModel::CoordinateFrameEnum::WORLD);

  Ogre::Vector3 ogre_position(position.x(), position.y(), position.z());

  visual_.setTransform(ogre_position);

  romo::Vector normal = contact->getNormal();

  Ogre::Vector3 ogre_normal(normal.x(), normal.y(), normal.z());

  visual_.setNormal(ogre_normal);

  romo::Force force = contact->getForce();

  Ogre::Vector3 ogre_force(force.x(), force.y(), force.z());

  visual_.setWrench(ogre_force);

  if (color_mode_ == ContactColorModeEnum::CONTACTPLANE) {
    Ogre::ColourValue color;
    switch (contact->getState()) {
      case romo::Contact<anymal_description::ConcreteAnymalDescription>::ContactState::OPEN:
        break;

      case romo::Contact<anymal_description::ConcreteAnymalDescription>::ContactState::SLIPPING:
        plane_color_ = Ogre::ColourValue::Red;
        plane_color_.a = alpha_property_->getFloat();
        visual_.setPlaneColor(plane_color_);
        break;

      case romo::Contact<anymal_description::ConcreteAnymalDescription>::ContactState::CLOSED:
        plane_color_ = Ogre::ColourValue::Green;
        plane_color_.a = alpha_property_->getFloat();
        visual_.setPlaneColor(plane_color_);
        break;

      default:
        plane_color_ = Ogre::ColourValue(0.5, 0.5, 0.5, alpha_property_->getFloat());
        visual_.setPlaneColor(plane_color_);
        break;
    }
  }
}

void Contact::enableContact() {
  visual_.setEnabled(contact_property_->getBool() && valid_draw_);
}

void Contact::changeColorMode() {
  color_mode_ = (ContactColorModeEnum)color_mode_property_->getOptionInt();
  if (color_mode_ == ContactColorModeEnum::SOLIDCOLOR) {
    changeColorAlpha();
  }
}

void Contact::changeColorAlpha() {
  Ogre::ColourValue color = rviz::qtToOgre(color_property_->getColor());
  color.a = alpha_property_->getFloat();
  visual_.setForceColor(color);
  if (color_mode_ == ContactColorModeEnum::SOLIDCOLOR) {
    plane_color_ = color;
  } else {
    plane_color_.a = alpha_property_->getFloat();
  }
  visual_.setPlaneColor(plane_color_);
}

void Contact::changeWrenchScale() {
  visual_.setForceScale(wrench_scale_property_->getFloat());
}

void Contact::changeWrenchWidth() {
  visual_.setArrowWidth(wrench_width_property_->getFloat());
}

void Contact::changePlaneSize() {
  visual_.setPlaneSize(plane_size_property_->getFloat());
}

void Contact::enableShowWrench() {
  visual_.setForceVisible(show_wrench_property_->getBool());
}

void Contact::enableShowNormal() {
  visual_.setPlaneVisible(show_normal_property_->getBool());
}

const std::string& Contact::getName() {
  return name_;
}

void Contact::updateEnabled(bool enabled) {
  contact_property_->setBool(enabled);
}

void Contact::updateColorMode(ContactColorModeEnum color_mode) {
  color_mode_property_->setValue(ContactColorModeMap.at(color_mode));
}

void Contact::updateColor(const QColor& color) {
  color_property_->setColor(color);
}

void Contact::updateAlpha(float alpha) {
  alpha_property_->setFloat(alpha);
}

void Contact::updateWrenchScale(float wrench_scale) {
  wrench_scale_property_->setFloat(wrench_scale);
}

void Contact::updateWrenchWidth(float wrench_width) {
  wrench_width_property_->setFloat(wrench_width);
}

void Contact::updatePlaneSize(float plane_size) {
  plane_size_property_->setValue(plane_size);
}

void Contact::enableShowWrench(bool enable_wrench) {
  show_wrench_property_->setBool(enable_wrench);
}

void Contact::enableShowNormal(bool enable_normal_plane) {
  show_normal_property_->setBool(enable_normal_plane);
}

}  // namespace anymal_rviz_plugin
