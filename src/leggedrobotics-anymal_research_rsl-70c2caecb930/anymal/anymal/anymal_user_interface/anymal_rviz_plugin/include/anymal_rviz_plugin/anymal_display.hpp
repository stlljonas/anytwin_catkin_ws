/*
 * anymal_display.hpp
 *
 *      Author: Perry Franklin
 */

#pragma once

#include <map>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/load_resource.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/robot/robot.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <anymal_msgs/AnymalState.h>

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/color_robot.hpp"
#include "anymal_rviz_plugin/trace_list.hpp"

#include "anymal_rviz_plugin/anymal_link_updater.hpp"
#include "anymal_rviz_plugin/contact_list.hpp"
#include "anymal_rviz_plugin/contact_polygon.hpp"
#include "anymal_rviz_plugin/joint_torque_tree.hpp"
#include "anymal_rviz_plugin/velocity.hpp"

namespace anymal_rviz_plugin {

typedef anymal_msgs::AnymalState AnymalDisplayMessageType;

class AnymalDisplay : public rviz::MessageFilterDisplay<AnymalDisplayMessageType> {
  Q_OBJECT
 public:
  AnymalDisplay();
  virtual ~AnymalDisplay();

  // Overrides from Display
  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;

  void clear();

 private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateAlpha();
  void updateRobotDescription();
  void changedWorldTfName();

 private:
  void load();

  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  bool updateWorldFrame();

  std::unique_ptr<ColorRobot> color_robot_;
  std::unique_ptr<JointTorqueTree> joint_torque_tree_;
  std::unique_ptr<ContactList> contact_list_;
  std::unique_ptr<Velocity> velocity_;
  std::unique_ptr<ContactPolygon> contact_polygon_;
  std::unique_ptr<TraceList> trace_list_;

  std::unique_ptr<AnymalLinkUpdater> anymal_link_updater_;

  std::string robot_description_;

  rviz::Property* robot_display_subfolder_;
  rviz::Property* visual_enabled_property_;
  rviz::Property* collision_enabled_property_;
  rviz::FloatProperty* update_rate_property_;
  rviz::StringProperty* robot_description_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::TfFrameProperty* world_frame_name_property_;

  void processMessage(const AnymalDisplayMessageType::ConstPtr& msg) override;
  ros::Time last_msg_time_;

  bool loaded_;

  QIcon status_icon_error_;
};

}  // namespace anymal_rviz_plugin
