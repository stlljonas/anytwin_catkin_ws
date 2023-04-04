/*
 * trace.hpp
 *
 *  Created on: Feb 25, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <QObject>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>

#include <anymal_rviz_plugin/anymal_link_updater.hpp>

namespace anymal_rviz_plugin {

class Trace : public QObject {
  Q_OBJECT
 public:
  Trace(const std::string& name, rviz::Property* parent_property, Ogre::SceneManager* manager, Ogre::SceneNode* scene_node);
  ~Trace();

  void update(double time, const AnymalLinkUpdater& updater);
  void drawVisual();

  void setEnabled(bool enabled);
  void setColor(const Ogre::ColourValue& color);
  void setTimeLength(double time_length);
  void setLineWidth(double width);

  void clearChain();

 private Q_SLOTS:
  void enableTrace();
  void changedColorAlpha();

 private:
  void trimPositionChain();

  const std::string name_;
  bool enabled_;
  double max_time_length_;

  // For this, the front should be the latest position, and the back should be the earliest position
  std::list<std::pair<double, Ogre::Vector3> > position_chain_;

  rviz::BillboardLine trace_visual_;

  rviz::BoolProperty* trace_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
};

}  // namespace anymal_rviz_plugin
