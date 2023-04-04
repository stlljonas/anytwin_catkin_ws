/*
 * trace_list.cpp
 *
 *  Created on: Feb 25, 2018
 *      Author: Perry Franklin
 */

#include <anymal_rviz_plugin/trace_list.hpp>

#include <anymal_description/AnymalDescription.hpp>

namespace anymal_rviz_plugin {

TraceList::TraceList(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name, rviz::Property* parent_property) {
  trace_list_property_ = new rviz::BoolProperty("Traces", true, "Enable all body traces", parent_property, SLOT(enableAllTraces()), this);

  time_length_property_ =
      new rviz::FloatProperty("Time Length", 10.0, "The maximum time length of the trace, IE how far in the past to draw the trace",
                              trace_list_property_, SLOT(changedTimeLength()), this);
  time_length_property_->setMin(0.0);

  clear_button_property_ = new rviz::ButtonProperty("Clear History", QString(), "Clears the history of all traces", trace_list_property_,
                                                    SLOT(clearHistory()), this);

  color_property_ = new rviz::ColorProperty("Color", QColor(255, 0, 0), "The color of the Trace visual", trace_list_property_,
                                            SLOT(changedColorAlpha()), this);

  alpha_property_ =
      new rviz::FloatProperty("Alpha", 0.5, "The transparency of the Trace visual", trace_list_property_, SLOT(changedColorAlpha()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  line_width_property_ =
      new rviz::FloatProperty("Width", 0.005, "The width of the Trace line", trace_list_property_, SLOT(changedLineWidth()), this);
  line_width_property_->setMin(0.0);

  for (const auto& key : anymal_description::AnymalTopology::bodyKeys) {
    trace_list_.emplace(key.getName(), std::make_unique<Trace>(key.getName(), trace_list_property_, context->getSceneManager(), root_node));
  }

  trace_list_property_->setDisableChildrenIfFalse(true);

  enableAllTraces();
  changedColorAlpha();
  changedTimeLength();
  changedLineWidth();
}

TraceList::~TraceList() {
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second.reset();
  }

  delete time_length_property_;
  delete color_property_;
  delete alpha_property_;
  delete line_width_property_;

  delete trace_list_property_;
}

void TraceList::update(double time, const AnymalLinkUpdater& updater) {
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second->update(time, updater);
  }
}

void TraceList::drawVisuals() {
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second->drawVisual();
  }
}

void TraceList::clearAllTraces() {
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second->clearChain();
  }
}

void TraceList::enableAllTraces() {
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second->setEnabled(trace_list_property_->getBool());
  }
}

void TraceList::changedTimeLength() {
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second->setTimeLength(time_length_property_->getFloat());
  }
}

void TraceList::changedColorAlpha() {
  Ogre::ColourValue color = rviz::qtToOgre(color_property_->getColor());
  color.a = alpha_property_->getFloat();
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second->setColor(color);
  }
}

void TraceList::changedLineWidth() {
  for (auto& name_trace_pair : trace_list_) {
    name_trace_pair.second->setLineWidth(line_width_property_->getFloat());
  }
}

void TraceList::clearHistory() {
  clearAllTraces();
}

}  // namespace anymal_rviz_plugin
