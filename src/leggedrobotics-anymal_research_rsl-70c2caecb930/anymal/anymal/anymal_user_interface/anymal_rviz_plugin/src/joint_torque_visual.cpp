/*
 * joint_torque_visual.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Perry Franklin
 */

#include <anymal_rviz_plugin/joint_torque_visual.hpp>

#include <ros/console.h>

namespace anymal_rviz_plugin {

JointTorqueVisual::JointTorqueVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager),
      frame_node_(parent_node->createChildSceneNode()),
      enabled_(true),
      complete_circle_enabled_(true),
      magnitude_type_(MagnitudeType::LENGTH),
      magnitude_(0.0),
      magnitude_max_(10.0),
      color_mode_(JointTorqueColorModeEnum::SOLID_COLOR),
      threshold_(0.75),
      alpha_(1.0),
      scale_(0.1),
      width_(0.01),
      offset_(0.0),
      axis_(Ogre::Vector3::ZERO),
      position_(Ogre::Vector3::ZERO),
      orientation_(Ogre::Quaternion::IDENTITY) {}

JointTorqueVisual::~JointTorqueVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void JointTorqueVisual::drawVisual() {
  if (enabled_) {
    const Ogre::Vector3 default_drawing_axis(0.0, 0.0, 1.0);

    Ogre::Quaternion axis_orientation = default_drawing_axis.getRotationTo(axis_, Ogre::Vector3(1.0, 0.0, 0.0));
    Ogre::Quaternion final_orientation = orientation_ * axis_orientation;

    if (!torque_circle_end_) {
      torque_circle_end_ = std::make_unique<rviz::Arrow>(scene_manager_, frame_node_);
    }
    if (!torque_circle_) {
      torque_circle_ = std::make_unique<rviz::BillboardLine>(scene_manager_, frame_node_);
    }
    if (!complete_circle_ && complete_circle_enabled_) {
      complete_circle_ = std::make_unique<rviz::BillboardLine>(scene_manager_, frame_node_);
    }

    double max_effort = 0.0;
    double abs_magnitude;

    if (max_effort != 0.0) {
      abs_magnitude = std::min(fabs(magnitude_) / max_effort, 1.0) + 0.05;
    } else {
      abs_magnitude = fabs(magnitude_) + 0.05;
    }

    const Ogre::ColourValue final_color = calculateColorValue(color_mode_, abs_magnitude, 0.0, magnitude_max_, color_);

    switch (magnitude_type_) {
      case MagnitudeType::SCALE: {
        torque_circle_end_->set(0, width_ * 2.0, width_ * 2.0 * 1.0, width_ * 2 * 2.0);
        if (magnitude_ > 0) {
          torque_circle_end_->setDirection(final_orientation * Ogre::Vector3(-1, 0, 0));
        } else {
          torque_circle_end_->setDirection(final_orientation * Ogre::Vector3(1, 0, 0));
        }
        torque_circle_end_->setPosition(final_orientation * Ogre::Vector3(0, 0.05 + abs_magnitude * scale_ * 0.5, 0) + position_);
        torque_circle_->clear();
        torque_circle_->setLineWidth(width_);
        for (int i = 0; i < 30; i++) {
          Ogre::Vector3 point = Ogre::Vector3((0.05 + abs_magnitude * scale_ * 0.5) * sin(i * 2 * M_PI / 32),
                                              (0.05 + abs_magnitude * scale_ * 0.5) * cos(i * 2 * M_PI / 32), 0);
          if (magnitude_ < 0) point.x = -point.x;
          torque_circle_->addPoint(final_orientation * point + position_);
        }
        torque_circle_end_->setColor(final_color.r, final_color.g, final_color.b, final_color.a);
        torque_circle_->setColor(final_color.r, final_color.g, final_color.b, final_color.a);

        break;
      }

      case MagnitudeType::LENGTH: {
        torque_circle_->clear();
        torque_circle_->setLineWidth(width_);
        torque_circle_end_->set(0, width_ * 2.0, width_ * 2.0 * 1.0, width_ * 2 * 2.0);

        if (complete_circle_enabled_) {
          complete_circle_->clear();
          complete_circle_->setLineWidth(width_ / 10.0);
          const double angle_delta = 0.1;
          for (double angle = 0.0; angle < 2 * M_PI; angle += angle_delta) {
            Ogre::Vector3 point = Ogre::Vector3(scale_ * cos(angle), scale_ * sin(angle), offset_);
            complete_circle_->addPoint(final_orientation * point + position_);
          }
          complete_circle_->addPoint(final_orientation * Ogre::Vector3(scale_ * cos(0.0), scale_ * sin(0.0), offset_) + position_);
        }

        const double max_angle = 2 * M_PI * magnitude_ / magnitude_max_;
        if (max_angle > 0) {
          const double angle_delta = 0.1;
          for (double angle = 0.0; angle < max_angle; angle += angle_delta) {
            Ogre::Vector3 point = Ogre::Vector3(scale_ * cos(angle), scale_ * sin(angle), offset_);
            torque_circle_->addPoint(final_orientation * point + position_);
          }
          torque_circle_->addPoint(final_orientation * Ogre::Vector3(scale_ * cos(max_angle), scale_ * sin(max_angle), offset_) +
                                   position_);
          torque_circle_end_->setDirection(final_orientation * Ogre::Quaternion(Ogre::Radian(max_angle), Ogre::Vector3(0, 0, 1.0)) *
                                           Ogre::Vector3(0, 1.0, 0));
        } else {
          const double angle_delta = -0.1;
          for (double angle = 0.0; angle > max_angle; angle += angle_delta) {
            Ogre::Vector3 point = Ogre::Vector3(scale_ * cos(angle), scale_ * sin(angle), offset_);
            torque_circle_->addPoint(final_orientation * point + position_);
          }
          torque_circle_->addPoint(final_orientation * Ogre::Vector3(scale_ * cos(max_angle), scale_ * sin(max_angle), offset_) +
                                   position_);

          torque_circle_end_->setDirection(final_orientation * Ogre::Quaternion(Ogre::Radian(max_angle), Ogre::Vector3(0, 0, 1.0)) *
                                           Ogre::Vector3(0, -1.0, 0));
        }

        torque_circle_->setColor(final_color.r, final_color.g, final_color.b, final_color.a);

        torque_circle_end_->setPosition(final_orientation * Ogre::Vector3(scale_ * cos(max_angle), scale_ * sin(max_angle), offset_) +
                                        position_);

        torque_circle_end_->setColor(final_color.r, final_color.g, final_color.b, final_color.a);

        complete_circle_->setColor(final_color.r, final_color.g, final_color.b, final_color.a);

        break;
      }

      default: {
        ROS_ERROR("joint_torque_visual.cpp: MagnitudeType not supported");
        break;
      }
    }
  }
}

void JointTorqueVisual::setTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  position_ = position;
  orientation_ = orientation;
}

void JointTorqueVisual::setMagnitude(double magnitude) {
  magnitude_ = magnitude;
}

void JointTorqueVisual::setMagnitudeMax(double magnitude_max) {
  magnitude_max_ = magnitude_max;
  drawVisual();
}

void JointTorqueVisual::setMagnitudeType(MagnitudeType magnitude_type) {
  magnitude_type_ = magnitude_type;
}

void JointTorqueVisual::setColorMode(JointTorqueColorModeEnum color_mode) {
  color_mode_ = color_mode;
  drawVisual();
}

void JointTorqueVisual::setThreshold(double threshold) {
  threshold_ = threshold;
  drawVisual();
}

void JointTorqueVisual::setColor(const Ogre::ColourValue& color) {
  color_ = color;
  drawVisual();
}

void JointTorqueVisual::setAlpha(float alpha) {
  alpha_ = alpha;
  drawVisual();
}

void JointTorqueVisual::setScale(float scale) {
  scale_ = scale;
  drawVisual();
}

void JointTorqueVisual::setWidth(float width) {
  width_ = width;
  drawVisual();
}

void JointTorqueVisual::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (enabled_ == false) {
    torque_circle_.reset();
    torque_circle_end_.reset();
    complete_circle_.reset();
  }
  drawVisual();
}

void JointTorqueVisual::setAxis(const Ogre::Vector3& axis) {
  axis_ = axis.normalisedCopy();
}

void JointTorqueVisual::setOffset(double offset) {
  offset_ = offset;
  drawVisual();
}

Ogre::ColourValue JointTorqueVisual::calculateColorValue(JointTorqueColorModeEnum color_mode, double magnitude, double magnitude_min,
                                                         double magnitude_max, const Ogre::ColourValue& reference_color) {
  Ogre::ColourValue final_color = reference_color;

  switch (color_mode) {
    case SOLID_COLOR:
      break;

    case COLOR_MAGNITUDE: {
      double inv_r = 1.0 - reference_color.r;
      double inv_g = 1.0 - reference_color.g;
      double inv_b = 1.0 - reference_color.b;

      double weight = (magnitude - magnitude_min) / (magnitude_max - magnitude_min);

      weight = std::min(weight, 1.0);
      weight = std::max(weight, 0.0);

      final_color.r = weight * reference_color.r + (1.0 - weight) * inv_r;
      final_color.g = weight * reference_color.g + (1.0 - weight) * inv_g;
      final_color.b = weight * reference_color.b + (1.0 - weight) * inv_b;
    } break;

    case COLOR_MAGNITUDE_RAINBOW:
      getRainbowColor(magnitude_min, magnitude_max, magnitude, final_color);
      break;

    case COLOR_MAGNITUDE_THRESHOLD: {
      double threshold_min = (1 - threshold_) * magnitude_min + threshold_ * magnitude_max;

      double weight = (magnitude - threshold_min) / (magnitude_max - threshold_min);

      weight = std::min(weight, 1.0);
      weight = std::max(weight, 0.0);

      final_color.r = (1.0 - weight) * reference_color.r + (weight)*1.0;
      final_color.g = (1.0 - weight) * reference_color.g;
      final_color.b = (1.0 - weight) * reference_color.b;
    } break;

    default:
      break;
  }

  return final_color;
}

void JointTorqueVisual::getRainbowColor(const double min, const double max, const double value, Ogre::ColourValue& color) {
  double adaptedValue = (value - min) / (max - min);

  // From rviz/src/rviz/default_plugin/point_cloud_transformers.cpp:
  adaptedValue = std::min(adaptedValue, 1.0);
  adaptedValue = std::max(adaptedValue, 0.0);

  float h = adaptedValue * 5.0 + 1.0;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1)) f = 1 - f;  // If i is even.
  float n = 1 - f;
  if (i <= 1)
    color.r = n, color.g = 0, color.b = 1;
  else if (i == 2)
    color.r = 0, color.g = n, color.b = 1;
  else if (i == 3)
    color.r = 0, color.g = 1, color.b = n;
  else if (i == 4)
    color.r = n, color.g = 1, color.b = 0;
  else if (i >= 5)
    color.r = 1, color.g = n, color.b = 0;
}

}  // namespace anymal_rviz_plugin
