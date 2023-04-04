/*
 * anymal_display.hpp
 *
 *      Author: Perry Franklin
 */

#include "anymal_rviz_plugin/anymal_display.hpp"

#include <tinyxml2.h>
#include <urdf/model.h>

#include <tf/transform_listener.h>

namespace anymal_rviz_plugin {

AnymalDisplay::AnymalDisplay() : loaded_(false) {
  status_icon_error_ = rviz::loadPixmap("package://rviz/icons/error.png");

  robot_description_property_ =
      new rviz::StringProperty("Robot Description", "anymal_description",
                               "Name of the parameter to search for to load the robot description.", this, SLOT(updateRobotDescription()));

  world_frame_name_property_ = new rviz::TfFrameProperty("World Frame Name", "odom", "Name of the Transform for the world frame", this,
                                                         nullptr, true, SLOT(changedWorldTfName()), this);

  update_rate_property_ = new rviz::FloatProperty("Update Rate", 20,
                                                  "Interval at which to update the links, in seconds. This is in ROS time, not wall time."
                                                  " 0 means to update on every message.",
                                                  this);
  update_rate_property_->setMin(0);

  robot_display_subfolder_ =
      new rviz::Property("Robot Display Options", QVariant(), "Options for controlling the robot visualization.", this);

  visual_enabled_property_ = new rviz::Property("Visual Enabled", true, "Whether to display the visual representation of the robot.",
                                                robot_display_subfolder_, SLOT(updateVisualVisible()), this);

  collision_enabled_property_ =
      new rviz::Property("Collision Enabled", false, "Whether to display the collision representation of the robot.",
                         robot_display_subfolder_, SLOT(updateCollisionVisible()), this);

  alpha_property_ = new rviz::FloatProperty("Alpha", 1, "Amount of transparency to apply to the links.", robot_display_subfolder_,
                                            SLOT(updateAlpha()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);
}

AnymalDisplay::~AnymalDisplay() {
  color_robot_.reset();
  joint_torque_tree_.reset();
  contact_list_.reset();
  velocity_.reset();
  contact_polygon_.reset();
  anymal_link_updater_.reset();
  trace_list_.reset();

  delete visual_enabled_property_;
  delete collision_enabled_property_;
  delete update_rate_property_;
  delete robot_description_property_;
  delete alpha_property_;
  delete world_frame_name_property_;
  delete robot_display_subfolder_;
}

void AnymalDisplay::onInitialize() {
  MFDClass::onInitialize();

  world_frame_name_property_->setFrameManager(context_->getFrameManager());

  color_robot_ = std::make_unique<ColorRobot>(scene_node_, context_, "Robot: " + getName().toStdString(), robot_display_subfolder_);
  joint_torque_tree_ = std::make_unique<JointTorqueTree>(scene_node_, context_, "JointTorqueTree: " + getName().toStdString(), this);
  contact_list_ = std::make_unique<ContactList>(scene_node_, context_, "ContactList: " + getName().toStdString(), this);
  velocity_ = std::make_unique<Velocity>(scene_node_, context_, "RobotVelocity: " + getName().toStdString(), this);
  contact_polygon_ = std::make_unique<ContactPolygon>(scene_node_, context_, "ContactPolygon: " + getName().toStdString(), this);
  trace_list_ = std::make_unique<TraceList>(scene_node_, context_, "TraceList: " + getName().toStdString(), this);

  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();

  robot_description_property_->setIcon(status_icon_error_);
}

void AnymalDisplay::updateAlpha() {
  color_robot_->getRobot().setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

void AnymalDisplay::updateRobotDescription() {
  if (isEnabled()) {
    load();
    context_->queueRender();
  }
}

void AnymalDisplay::updateVisualVisible() {
  color_robot_->getRobot().setVisualVisible(visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void AnymalDisplay::updateCollisionVisible() {
  color_robot_->getRobot().setCollisionVisible(collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void AnymalDisplay::load() {
  robot_description_property_->setIcon(status_icon_error_);

  std::string content;
  if (!update_nh_.getParam(robot_description_property_->getStdString(), content)) {
    std::string loc;
    if (update_nh_.searchParam(robot_description_property_->getStdString(), loc)) {
      update_nh_.getParam(loc, content);
    } else {
      clear();
      setStatus(rviz::StatusProperty::Error, "URDF",
                "Parameter [" + robot_description_property_->getString() + "] does not exist, and was not found by searchParam()");
      return;
    }
  }

  if (content.empty()) {
    clear();
    setStatus(rviz::StatusProperty::Error, "URDF", "URDF is empty");
    return;
  }

  if (content == robot_description_) {
    return;
  }

  robot_description_ = content;

  tinyxml2::XMLDocument doc;
  doc.Parse(robot_description_.c_str());
  if (!doc.RootElement()) {
    clear();
    setStatus(rviz::StatusProperty::Error, "URDF", "URDF failed XML parse");
    return;
  }

  urdf::Model descr;
  if (!descr.initXml(doc.RootElement())) {
    clear();
    setStatus(rviz::StatusProperty::Error, "URDF", "URDF failed Model parse");
    return;
  }

  anymal_link_updater_ = std::make_unique<AnymalLinkUpdater>(robot_description_, descr);

  setStatus(rviz::StatusProperty::Ok, "URDF", "URDF parsed OK");
  color_robot_->getRobot().load(descr);
  color_robot_->getRobot().update(*anymal_link_updater_);
  joint_torque_tree_->load_urdf(descr);
  joint_torque_tree_->update(*anymal_link_updater_);

  loaded_ = true;
  robot_description_property_->setIcon(QIcon());
}

void AnymalDisplay::onEnable() {
  load();
  color_robot_->getRobot().setVisible(true);
}

void AnymalDisplay::onDisable() {
  color_robot_->getRobot().setVisible(false);
}

void AnymalDisplay::processMessage(const AnymalDisplayMessageType::ConstPtr& msg) {
  if (loaded_) {
    ros::Duration time_since_last_transform_ = msg->header.stamp - last_msg_time_;
    float rate = update_rate_property_->getFloat();
    double period = 1.0 / rate;
    bool update = period < 0.0001f || time_since_last_transform_.toSec() >= period;

    if (time_since_last_transform_.toSec() < -0.0) {
      ROS_ERROR_STREAM("Looks like you went back in time. Nice!");
      update = true;
      trace_list_->clearAllTraces();
    }

    if (update) {
      if (updateWorldFrame()) {
        anymal_link_updater_->update(*msg);
        trace_list_->update(msg->header.stamp.toSec(), *anymal_link_updater_);

        if (isEnabled()) {
          color_robot_->getRobot().update(*anymal_link_updater_);
          joint_torque_tree_->update(*anymal_link_updater_);
          contact_polygon_->update(*anymal_link_updater_);
          contact_list_->update(*anymal_link_updater_);
          velocity_->update(*anymal_link_updater_);
          trace_list_->drawVisuals();
        }
      }

      last_msg_time_ = msg->header.stamp;
    }
  }
}

void AnymalDisplay::fixedFrameChanged() {
  // I'm not sure why this needs to be done. If it is removed, then loading the plugin while the
  // fixed frame is on an invalid frame causes the plugin to constantly have an error, created
  // by the tf_filter_.
  {
    delete tf_filter_;
    tf_filter_ =
        new tf2_ros::MessageFilter<AnymalDisplayMessageType>(*context_->getTF2BufferPtr(), fixed_frame_.toStdString(), 10, update_nh_);

    tf_filter_->connectInput(sub_);
    tf_filter_->registerCallback(boost::bind(&AnymalDisplay::incomingMessage, this, _1));
    context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
  }

  updateWorldFrame();
}

void AnymalDisplay::changedWorldTfName() {
  updateWorldFrame();
}

void AnymalDisplay::clear() {
  color_robot_->getRobot().clear();
  robot_description_.clear();
  anymal_link_updater_.reset();
  loaded_ = false;
}

void AnymalDisplay::reset() {
  tf_filter_->clear();
  messages_received_ = 0;
}

bool AnymalDisplay::updateWorldFrame() {
  Ogre::Vector3 world_position(Ogre::Vector3::ZERO);
  Ogre::Quaternion world_orientation(Ogre::Quaternion::IDENTITY);
  if (context_->getFrameManager()->getTransform(world_frame_name_property_->getStdString(), last_msg_time_, world_position,
                                                world_orientation)) {
    scene_node_->setPosition(world_position);
    scene_node_->setOrientation(world_orientation);
    setStatus(rviz::StatusProperty::Ok, "World TF",
              "Fixed Frame connects to world frame '" + world_frame_name_property_->getString() + "'");
    world_frame_name_property_->setIcon(QIcon());
    return true;
  } else {
    setStatus(rviz::StatusProperty::Error, "World TF",
              "Fixed Frame is not connected to world frame '" + world_frame_name_property_->getString() + "'");
    world_frame_name_property_->setIcon(status_icon_error_);
    return false;
  }
}

}  // namespace anymal_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(anymal_rviz_plugin::AnymalDisplay, rviz::Display)
