/*
 * contact_polygon.hpp
 *
 *  Created on: Jan 9, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <QObject>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/com_projection_visual.hpp"
#include "anymal_rviz_plugin/polygon_visual.hpp"

namespace anymal_rviz_plugin {

class ContactPolygon : QObject {
  Q_OBJECT
 public:
  ContactPolygon(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property);
  ~ContactPolygon();

  void update(const AnymalLinkUpdater& updater);

 private Q_SLOTS:
  void enableContactPolygon();
  void changeLineColorAlpha();
  void changeLineThickness();
  void enableComArrow();
  void changeComArrowColorAlpha();
  void enableComColumn();
  void changeComArrowThickness();
  void changeComPolygonProjectionColorAlpha();

 private:
  std::vector<Ogre::Vector3> giftWrap(const std::vector<Ogre::Vector3>& points);

  std::vector<Ogre::Vector3> convexHull(const std::vector<Ogre::Vector3>& points);

  rviz::BoolProperty* contact_polygon_property_;
  rviz::ColorProperty* line_color_property_;
  rviz::FloatProperty* line_alpha_property_;
  rviz::FloatProperty* line_thickness_property_;
  rviz::BoolProperty* show_com_projection_property_;
  rviz::ColorProperty* com_projection_color_property_;
  rviz::FloatProperty* com_projection_alpha_property_;
  rviz::BoolProperty* show_com_projection_column_property_;
  rviz::FloatProperty* com_projection_thickness_property_;
  rviz::ColorProperty* com_polygon_projection_color_property_;
  rviz::FloatProperty* com_polygon_projection_alpha_property_;

  PolygonVisual polygon_visual_;
  ComProjectionVisual com_projection_visual_;
  PolygonVisual com_polygon_visual_;
};

}  // namespace anymal_rviz_plugin
