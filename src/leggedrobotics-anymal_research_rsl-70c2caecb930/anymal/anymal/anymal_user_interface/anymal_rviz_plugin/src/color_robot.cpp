/*
 * color_robot.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: Perry Franklin
 */

#include <rviz/robot/robot_link.h>

#include "anymal_rviz_plugin/color_robot.hpp"

namespace anymal_rviz_plugin {

ColorRobotModeProperty::ColorRobotModeProperty(const QString& name, ColorRobotModeEnum default_mode, const QString& description,
                                               Property* parent, const char* changed_slot, QObject* receiver)
    : rviz::EnumProperty(name, ColorRobotModeMap.at(default_mode), description, parent, changed_slot, receiver) {
  for (const std::pair<int, QString>& num_option_pair : ColorRobotModeMap) {
    this->addOption(num_option_pair.second, num_option_pair.first);
  }
}

ColorRobot::~ColorRobot() {
  delete color_property_;
  delete color_mode_property_;
}

ColorRobot::ColorRobot(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property)
    : robot_(root_node, context, name, parent_property), options_(ColorRobotModeEnum::MESH_TEXTURE), color_(Ogre::ColourValue::White) {
  color_mode_property_ =
      new ColorRobotModeProperty("Color Mode", ColorRobotModeEnum::MESH_TEXTURE, "Sets the color mode for the robot visualization",
                                 parent_property, SLOT(changedColorMode()), this);

  color_property_ =
      new rviz::ColorProperty("Color", QColor(204, 204, 204), "Color of the Robot", color_mode_property_, SLOT(changedColor()), this);

  changedColorMode();
}

void ColorRobot::setColorMode(ColorRobotModeEnum color_mode) {
  switch (color_mode) {
    case ColorRobotModeEnum::MESH_TEXTURE:
      for (rviz::Robot::M_NameToLink::value_type name_link_pair : robot_.getLinks()) {
        name_link_pair.second->unsetColor();
      }
      break;
    case ColorRobotModeEnum::SOLID_COLOR:
      for (rviz::Robot::M_NameToLink::value_type name_link_pair : robot_.getLinks()) {
        name_link_pair.second->setColor(color_.r, color_.g, color_.b);
      }
      break;
    default:
      ROS_ERROR("ColorRobot Mode not supported");
  }
}

rviz::Robot& ColorRobot::getRobot() {
  return robot_;
}

void ColorRobot::changedColor() {
  color_ = rviz::qtToOgre(color_property_->getColor());
  setColorMode(ColorRobotModeEnum::SOLID_COLOR);
  color_mode_property_->setValue(ColorRobotModeMap.at(ColorRobotModeEnum::SOLID_COLOR));
}

void ColorRobot::changedColorMode() {
  ColorRobotModeEnum current_mode = (ColorRobotModeEnum)color_mode_property_->getOptionInt();

  if (current_mode == ColorRobotModeEnum::SOLID_COLOR) {
    color_mode_property_->expand();
  }

  setColorMode(current_mode);
}

}  // namespace anymal_rviz_plugin
