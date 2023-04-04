/*
 * contact.hpp
 *
 *  Created on: Jan 1, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <QObject>

#include <OgreColourValue.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/contact_visual.hpp"

namespace anymal_rviz_plugin {

typedef enum { SOLIDCOLOR = 0, CONTACTPLANE, CONTACTLEG } ContactColorModeEnum;

const std::map<int, QString> ContactColorModeMap = {{ContactColorModeEnum::SOLIDCOLOR, "Solid Color"},
                                                    {ContactColorModeEnum::CONTACTPLANE, "Contact/Plane"}};
//                                                    {ContactColorModeEnum::CONTACTLEG,"Contact/Leg"}};

// Helper class for creating an EnumProperty with the same enum list every time.
class ContactOptionProperty : public rviz::EnumProperty {
 public:
  ContactOptionProperty(const QString& name, ContactColorModeEnum default_mode, const QString& description, rviz::Property* parent,
                        const char* changed_slot, QObject* receiver);
};

class Contact : QObject {
  Q_OBJECT
 public:
  Contact(const std::string& name, rviz::Property* parent_property, Ogre::SceneManager* manager, Ogre::SceneNode* scene_node_);
  virtual ~Contact();

  void setColorMode();

  const std::string& getName();

  void update(const AnymalLinkUpdater& updater);

 private Q_SLOTS:
  void enableContact();
  void changeColorMode();
  void changeColorAlpha();
  void changeWrenchScale();
  void changeWrenchWidth();
  void changePlaneSize();
  void enableShowWrench();
  void enableShowNormal();

 public:  // Functions which directly modify the visual QT objects
  void updateEnabled(bool enabled);
  void updateColorMode(ContactColorModeEnum color_mode);
  void updateColor(const QColor& color);
  void updateAlpha(float alpha);
  void updateWrenchScale(float wrench_scale);
  void updateWrenchWidth(float wrench_width);
  void updatePlaneSize(float plane_size);
  void enableShowWrench(bool enable_wrench);
  void enableShowNormal(bool enable_normal_plane);

 private:
  Ogre::ColourValue plane_color_;

  ContactColorModeEnum color_mode_;

  const std::string name_;

  bool valid_draw_;
  ContactVisual visual_;

  rviz::BoolProperty* contact_property_;
  rviz::EnumProperty* color_mode_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* wrench_scale_property_;
  rviz::FloatProperty* wrench_width_property_;
  rviz::FloatProperty* plane_size_property_;
  rviz::BoolProperty* show_wrench_property_;
  rviz::BoolProperty* show_normal_property_;
};

}  // namespace anymal_rviz_plugin
