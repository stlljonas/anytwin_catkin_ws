/*
 * trace_list.hpp
 *
 *  Created on: Feb 25, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <QObject>
#include <anymal_rviz_plugin/trace.hpp>

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include "rviz/properties/button_property.h"

#include "anymal_rviz_plugin/anymal_link_updater.hpp"

namespace anymal_rviz_plugin {

class TraceList : public QObject {
  Q_OBJECT
 public:
  TraceList(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property);
  ~TraceList();

  void update(double time, const AnymalLinkUpdater& updater);
  void drawVisuals();

  void clearAllTraces();

 private Q_SLOTS:

  void enableAllTraces();
  void changedTimeLength();
  void changedColorAlpha();
  void changedLineWidth();
  void clearHistory();

 private:
  std::map<std::string, std::unique_ptr<Trace> > trace_list_;

  rviz::BoolProperty* trace_list_property_;
  rviz::FloatProperty* time_length_property_;
  rviz::ButtonProperty* clear_button_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* line_width_property_;
};

}  // namespace anymal_rviz_plugin
