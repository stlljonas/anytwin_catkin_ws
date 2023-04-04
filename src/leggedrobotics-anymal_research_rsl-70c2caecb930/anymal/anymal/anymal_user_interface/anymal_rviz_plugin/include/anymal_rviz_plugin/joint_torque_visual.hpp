/*
 * joint_torque_visual.hpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <memory>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

namespace anymal_rviz_plugin {

class JointTorqueVisual {
 public:
  typedef enum { SOLID_COLOR = 0, COLOR_MAGNITUDE, COLOR_MAGNITUDE_RAINBOW, COLOR_MAGNITUDE_THRESHOLD } JointTorqueColorModeEnum;

 public:
  enum MagnitudeType { SCALE = 0, LENGTH };

  JointTorqueVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~JointTorqueVisual();

  void drawVisual();

  void setTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

  void setMagnitude(double magnitude);

  void setMagnitudeMax(double magnitude_max);

  void setMagnitudeType(MagnitudeType magnitude_type);

  void setColorMode(JointTorqueColorModeEnum color_mode);

  void setThreshold(double threshold);

  void setColor(const Ogre::ColourValue& color);

  void setAlpha(float alpha);

  void setScale(float scale);

  void setWidth(float width);

  void setEnabled(bool enabled);

  void setAxis(const Ogre::Vector3& axis);

  void setOffset(double offset);

  void setCompleteCircle(bool complete_circle_enabled);

 private:
  Ogre::ColourValue calculateColorValue(JointTorqueColorModeEnum color_mode, double magnitude, double magnitude_min, double magnitude_max,
                                        const Ogre::ColourValue& reference_color);

  void getRainbowColor(const double min, const double max, const double value, Ogre::ColourValue& color);

  Ogre::SceneManager* const scene_manager_;
  Ogre::SceneNode* const frame_node_;

  bool enabled_;
  bool complete_circle_enabled_;

  MagnitudeType magnitude_type_;
  double magnitude_;
  double magnitude_max_;

  JointTorqueColorModeEnum color_mode_;
  double threshold_;

  Ogre::ColourValue color_;
  float alpha_;
  float scale_;
  float width_;
  double offset_;

  Ogre::Vector3 axis_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  std::unique_ptr<rviz::BillboardLine> complete_circle_;
  std::unique_ptr<rviz::BillboardLine> torque_circle_;
  std::unique_ptr<rviz::Arrow> torque_circle_end_;
};

}  // namespace anymal_rviz_plugin
