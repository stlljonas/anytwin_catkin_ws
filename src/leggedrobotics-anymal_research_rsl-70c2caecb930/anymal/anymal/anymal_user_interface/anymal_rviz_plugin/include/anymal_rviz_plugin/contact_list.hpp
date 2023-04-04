/*
 * contact_list.hpp
 *
 *  Created on: Dec 30, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <rviz/display.h>

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <OgreAny.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <QObject>

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/contact.hpp"

namespace anymal_rviz_plugin {

class ContactList : public QObject {
  Q_OBJECT
 public:
  ContactList(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property);
  virtual ~ContactList();

  void update(const AnymalLinkUpdater& updater);

  Ogre::SceneManager* getSceneManager();
  Ogre::SceneNode* getSceneNode();

 private Q_SLOTS:
  void enableAllContacts();
  void changeEnabledAll();
  void changeColorModeAll();
  void changeColorAll();
  void changeAlphaAll();
  void changeWrenchScaleAll();
  void changeWrenchWidthAll();
  void changePlaneSizeAll();
  void enableShowWrenchAll();
  void enableShowNormalAll();

 private:
  Ogre::SceneManager* const scene_manager_;
  Ogre::SceneNode* const scene_node_;
  rviz::BoolProperty* contact_list_property_;
  rviz::EnumProperty* color_mode_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* wrench_scale_property_;
  rviz::FloatProperty* wrench_width_property_;
  rviz::FloatProperty* plane_size_property_;
  rviz::BoolProperty* show_wrench_property_;
  rviz::BoolProperty* show_normal_property_;

  std::map<std::string, std::unique_ptr<Contact> > contacts_visuals_list_;
};

}  // namespace anymal_rviz_plugin
