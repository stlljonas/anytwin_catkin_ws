/*!
 * @authors         Aravind Vijayan
 * @affiliation     ANYbotics
 * @brief           Implementation of Robot re-spawing tool.
 * @date            November, 2019
 */

#include "rviz_gazebo_interaction/RobotRespawner.hpp"

// any gazebo msgs
#include <any_gazebo_msgs/SetRobotPose.h>

// pluginlib
#include <pluginlib/class_list_macros.h>

// rviz
#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/selection/selection_manager.h>

namespace rviz_gazebo_interaction {

RobotRespawner::RobotRespawner() {
  shortcut_key_ = 's';
}

void RobotRespawner::onInitialize() {
  markerPub_ = nh_.advertise<visualization_msgs::Marker>("robot_respawner_marker", 1);
  gazeboSetRobotPose_ = nh_.serviceClient<any_gazebo_msgs::SetRobotPose>("/gazebo/set_robot_pose");
  deactivate();
}

void RobotRespawner::activate() {
  context_->getSelectionManager()->setTextureSize(1);
  arrowMarker_.header.frame_id = context_->getFixedFrame().toStdString();
  arrowMarker_.type = visualization_msgs::Marker::ARROW;
  arrowMarker_.header.stamp = ros::Time::now();
  arrowMarker_.ns = "arrow";
  arrowMarker_.id = 0;
  arrowMarker_.color.r = 1.0f;
  arrowMarker_.color.g = 0.0f;
  arrowMarker_.color.b = 0.0f;
  arrowMarker_.color.a = 1.0;
  arrowMarker_.lifetime = ros::Duration();
}

void RobotRespawner::deactivate() {}

int RobotRespawner::processMouseEvent(rviz::ViewportMouseEvent& event) {
  const int flags = 0;
  if (event.panel->contextMenuVisible()) {
    return flags;
  }

  if (event.leftDown()) {
    if (!setStartPosition(event)) {
      return flags;
    }
  }
  if (event.left()) {
    visualizeOrientation(event);
  }

  if (event.type == QEvent::MouseButtonRelease) {
    arrowMarker_.action = visualization_msgs::Marker::DELETE;
    markerPub_.publish(arrowMarker_);
    respawnAtPosition(event);
    return Finished;
  }
  return flags;
}

bool RobotRespawner::setStartPosition(const rviz::ViewportMouseEvent& event) {
  startScreenPosition_ = Ogre::Vector2(event.x, event.y);
  return context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, spawnPosition_);
}

void RobotRespawner::visualizeOrientation(const rviz::ViewportMouseEvent& event) {
  const Ogre::Vector2 endScreenPosition = Ogre::Vector2(event.x, event.y);
  const auto theta = std::atan2((endScreenPosition - startScreenPosition_).y, (endScreenPosition - startScreenPosition_).x);
  Ogre::Quaternion orientation;
  orientation.w = std::cos(-0.5 * theta);
  orientation.z = std::sin(-0.5 * theta);
  orientation.normalise();
  spawnOrientation_ = orientation;
  arrowMarker_.action = visualization_msgs::Marker::ADD;
  arrowMarker_.pose.position.x = spawnPosition_.x;
  arrowMarker_.pose.position.y = spawnPosition_.y;
  arrowMarker_.pose.position.z = spawnPosition_.z;
  arrowMarker_.pose.orientation.x = orientation.x;
  arrowMarker_.pose.orientation.y = orientation.y;
  arrowMarker_.pose.orientation.z = orientation.z;
  arrowMarker_.pose.orientation.w = orientation.w;

  arrowMarker_.scale.x = 0.7;
  arrowMarker_.scale.y = 0.05;
  arrowMarker_.scale.z = 0.05;
  markerPub_.publish(arrowMarker_);
}

bool RobotRespawner::respawnAtPosition(const rviz::ViewportMouseEvent& event) {
  any_gazebo_msgs::SetRobotPose robotPose;
  robotPose.request.pose.position.x = spawnPosition_.x;
  robotPose.request.pose.position.y = spawnPosition_.y;
  robotPose.request.pose.position.z = spawnPosition_.z + 1.0;
  robotPose.request.pose.orientation.x = spawnOrientation_.x;
  robotPose.request.pose.orientation.y = spawnOrientation_.y;
  robotPose.request.pose.orientation.z = spawnOrientation_.z;
  robotPose.request.pose.orientation.w = spawnOrientation_.w;
  return gazeboSetRobotPose_.call(robotPose);
}

}  // namespace rviz_gazebo_interaction

PLUGINLIB_EXPORT_CLASS(rviz_gazebo_interaction::RobotRespawner, rviz::Tool)
