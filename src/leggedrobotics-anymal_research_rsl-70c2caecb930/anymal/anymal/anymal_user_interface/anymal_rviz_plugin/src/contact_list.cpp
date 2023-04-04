/*
 * contact_list.cpp
 *
 *  Created on: Jan 1, 2018
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/contact_list.hpp"

#include <anymal_description/AnymalDescription.hpp>

namespace anymal_rviz_plugin {

ContactList::ContactList(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name,
                         rviz::Property* parent_property)
    : scene_manager_(context->getSceneManager()), scene_node_(root_node->createChildSceneNode()) {
  contact_list_property_ =
      new rviz::BoolProperty("Contacts", true, "Enable all contacts", parent_property, SLOT(enableAllContacts()), this);

  color_mode_property_ = new ContactOptionProperty("Color Mode", ContactColorModeEnum::CONTACTPLANE, "", contact_list_property_,
                                                   SLOT(changeColorModeAll()), this);

  color_property_ =
      new rviz::ColorProperty("Color", QColor(100, 100, 100), "Color of the contact", contact_list_property_, SLOT(changeColorAll()), this);

  alpha_property_ =
      new rviz::FloatProperty("Alpha", 0.3, "Alpha (transparency) of the contact", contact_list_property_, SLOT(changeAlphaAll()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  wrench_scale_property_ = new rviz::FloatProperty("Wrench Scale", 0.003, "Scale of the force/torque", contact_list_property_,
                                                   SLOT(changeWrenchScaleAll()), this);
  wrench_scale_property_->setMin(0.0);

  wrench_width_property_ = new rviz::FloatProperty("Wrench Arrow Width", 0.01, "Width of the force/torque arrow", contact_list_property_,
                                                   SLOT(changeWrenchWidthAll()), this);
  wrench_width_property_->setMin(0.0);

  plane_size_property_ = new rviz::FloatProperty("Normal Plane Size", 0.2, "Size of the of the contact normal plane",
                                                 contact_list_property_, SLOT(changePlaneSizeAll()), this);
  plane_size_property_->setMin(0.0);

  show_wrench_property_ = new rviz::BoolProperty("Show Wrench", true, "Show the wrench (force/torque) visual for the contact",
                                                 contact_list_property_, SLOT(enableShowWrenchAll()), this);

  show_normal_property_ = new rviz::BoolProperty("Show Normal", true, "Show the normal plane of the contact", contact_list_property_,
                                                 SLOT(enableShowNormalAll()), this);

  contact_list_property_->setDisableChildrenIfFalse(true);

  for (auto contact : anymal_description::AnymalTopology::contactKeys) {
    contacts_visuals_list_.emplace(contact.getName(),
                                   std::make_unique<Contact>(contact.getName(), contact_list_property_, scene_manager_, scene_node_));
  }
}

ContactList::~ContactList() {
  for (auto& element : contacts_visuals_list_) {
    element.second.reset();
  }

  delete color_mode_property_;
  delete color_property_;
  delete alpha_property_;
  delete wrench_scale_property_;
  delete wrench_width_property_;
  delete plane_size_property_;
  delete show_wrench_property_;
  delete show_normal_property_;

  delete contact_list_property_;
}

void ContactList::update(const AnymalLinkUpdater& updater) {
  for (auto& element : contacts_visuals_list_) {
    element.second->update(updater);
  }
}

Ogre::SceneManager* ContactList::getSceneManager() {
  return scene_manager_;
}

Ogre::SceneNode* ContactList::getSceneNode() {
  return scene_node_;
}

void ContactList::enableAllContacts() {
  for (auto& element : contacts_visuals_list_) {
    element.second->updateEnabled(contact_list_property_->getBool());
  }
}

void ContactList::changeColorModeAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->updateColorMode((ContactColorModeEnum)color_mode_property_->getOptionInt());
  }
}

void ContactList::changeColorAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->updateColor(color_property_->getColor());
  }
}

void ContactList::changeAlphaAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->updateAlpha(alpha_property_->getFloat());
  }
}

void ContactList::changeWrenchScaleAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->updateWrenchScale(wrench_scale_property_->getFloat());
  }
}

void ContactList::changeWrenchWidthAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->updateWrenchWidth(wrench_width_property_->getFloat());
  }
}

void ContactList::changePlaneSizeAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->updatePlaneSize(plane_size_property_->getFloat());
  }
}

void ContactList::enableShowWrenchAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->enableShowWrench(show_wrench_property_->getBool());
  }
}

void ContactList::enableShowNormalAll() {
  for (auto& element : contacts_visuals_list_) {
    element.second->enableShowNormal(show_normal_property_->getBool());
  }
}

}  // namespace anymal_rviz_plugin
