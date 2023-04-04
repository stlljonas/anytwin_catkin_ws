/*!
 * @authors         Aravind Vijayan
 * @affiliation     ANYbotics
 * @brief           Declaration of Robot re-spawing tool.
 * @date            November, 2019
 */

#pragma once
// OGRE
#include <Ogre.h>

// ros
#include <ros/ros.h>
#include <rviz/tool.h>

// visualization msgs
#include <visualization_msgs/Marker.h>

namespace rviz_gazebo_interaction {

class RobotRespawner : public rviz::Tool {
  Q_OBJECT
 public:
  RobotRespawner();
  ~RobotRespawner() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

 private:
  bool setStartPosition(const rviz::ViewportMouseEvent& event);
  void visualizeOrientation(const rviz::ViewportMouseEvent& event);
  bool respawnAtPosition(const rviz::ViewportMouseEvent& event);

  ros::NodeHandle nh_;
  ros::ServiceClient gazeboSetRobotPose_;
  Ogre::Vector2 startScreenPosition_;
  Ogre::Vector3 spawnPosition_;
  Ogre::Quaternion spawnOrientation_;
  ros::Publisher markerPub_;
  visualization_msgs::Marker arrowMarker_;
};

}  // namespace rviz_gazebo_interaction
