/*
 * trace.cpp
 *
 *  Created on: Feb 25, 2018
 *      Author: Perry Franklin
 */

#include <anymal_rviz_plugin/trace.hpp>

namespace anymal_rviz_plugin {

Trace::Trace(const std::string& name, rviz::Property* parent_property, Ogre::SceneManager* manager, Ogre::SceneNode* scene_node)
    : name_(name), enabled_(true), max_time_length_(1.0), trace_visual_(manager, scene_node) {
  trace_property_ =
      new rviz::BoolProperty(name.c_str(), false, "Enables the Trace visual for this body", parent_property, SLOT(enableTrace()), this);

  color_property_ = new rviz::ColorProperty("Color", QColor(255, 0.0, 0.0), "Color of the Trace visual for this body", trace_property_,
                                            SLOT(changedColorAlpha()), this);

  alpha_property_ =
      new rviz::FloatProperty("Alpha", 0.5, "Alpha of the Trace visual for this body", trace_property_, SLOT(changedColorAlpha()), this);

  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  trace_visual_.setLineWidth(0.01);

  trace_property_->setDisableChildrenIfFalse(true);

  changedColorAlpha();
}

Trace::~Trace() {}

void Trace::update(double time, const AnymalLinkUpdater& updater) {
  if (position_chain_.empty()) {
    const anymal_model::AnymalModel& model = updater.getModel();
    anymal_model::AnymalModel::BodyEnum body_enum = anymal_description::AnymalTopology::bodyKeys.atName(name_).getEnum();

    kindr::Position3D new_position(model.getPositionWorldToBody(body_enum, anymal_model::AnymalModel::CoordinateFrameEnum::WORLD));

    Ogre::Vector3 new_position_ogre(new_position.x(), new_position.y(), new_position.z());

    position_chain_.push_front(std::make_pair(time, new_position_ogre));
  } else if (time > position_chain_.front().first) {
    const anymal_model::AnymalModel& model = updater.getModel();
    anymal_model::AnymalModel::BodyEnum body_enum = anymal_description::AnymalTopology::bodyKeys.atName(name_).getEnum();

    kindr::Position3D new_position(model.getPositionWorldToBody(body_enum, anymal_model::AnymalModel::CoordinateFrameEnum::WORLD));

    Ogre::Vector3 new_position_ogre(new_position.x(), new_position.y(), new_position.z());

    position_chain_.push_front(std::make_pair(time, new_position_ogre));

    trimPositionChain();
  } else {
    ROS_WARN_THROTTLE(1, "trace.cpp: You've tried to add a new time point to a trace but it is in the past");
  }
}

void Trace::drawVisual() {
  trace_visual_.clear();

  if (enabled_ && trace_property_->getBool()) {
    trace_visual_.setMaxPointsPerLine(position_chain_.size());

    double current_time = position_chain_.front().first;
    for (const std::pair<double, Ogre::Vector3>& time_position_pair : position_chain_) {
      if (current_time - time_position_pair.first > max_time_length_) {
        break;
      }
      trace_visual_.addPoint(time_position_pair.second);
    }
  }
}

void Trace::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (enabled_ == false) {
    trace_visual_.clear();
  }
  if (enabled_ == true) {
    drawVisual();
  }
}

void Trace::setColor(const Ogre::ColourValue& color) {
  trace_visual_.setColor(color.r, color.g, color.b, color.a);
}

void Trace::setTimeLength(double time_length) {
  max_time_length_ = time_length;
  drawVisual();
}

void Trace::setLineWidth(double width) {
  trace_visual_.setLineWidth(width);
}

void Trace::clearChain() {
  position_chain_.clear();
}

void Trace::changedColorAlpha() {
  Ogre::ColourValue color = rviz::qtToOgre(color_property_->getColor());
  color.a = alpha_property_->getFloat();
  setColor(color);
}

void Trace::trimPositionChain() {
  if (!position_chain_.empty()) {
    const double latest_time = position_chain_.front().first;
    while (latest_time - position_chain_.back().first > max_time_length_) {
      position_chain_.pop_back();
    }
  }
}

void Trace::enableTrace() {
  setEnabled(enabled_);
}

}  // namespace anymal_rviz_plugin
