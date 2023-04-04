/*
 * color_robot.hpp
 *
 *  Created on: Feb 9, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <OgreSceneNode.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/robot/link_updater.h>
#include <rviz/robot/robot.h>

namespace anymal_rviz_plugin {

typedef enum { MESH_TEXTURE = 0, SOLID_COLOR } ColorRobotModeEnum;

const std::map<int, QString> ColorRobotModeMap = {{ColorRobotModeEnum::MESH_TEXTURE, "Mesh Texture"},
                                                  {ColorRobotModeEnum::SOLID_COLOR, "Solid Color"}};

// Helper class for creating an EnumProperty with the same enum list every time.
class ColorRobotModeProperty : public rviz::EnumProperty {
 public:
  ColorRobotModeProperty(const QString& name, ColorRobotModeEnum default_mode, const QString& description, Property* parent,
                         const char* changed_slot, QObject* receiver);
};

/**
 * The ColorRobot class encapuslates a rviz::Robot and adds options for setting the color of the links
 */
class ColorRobot : public QObject {
  Q_OBJECT
 public:
  ColorRobot(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property);
  ~ColorRobot();

  void setToSolidColor(const Ogre::ColourValue& color_value);
  void setToMeshTexture();

  void setColorMode(ColorRobotModeEnum color_mode);

  rviz::Robot& getRobot();

 private Q_SLOTS:

  void changedColorMode();
  void changedColor();

 private:
  rviz::Robot robot_;

  ColorRobotModeEnum options_;
  Ogre::ColourValue color_;

  rviz::EnumProperty* color_mode_property_;
  rviz::ColorProperty* color_property_;
};

}  // namespace anymal_rviz_plugin
