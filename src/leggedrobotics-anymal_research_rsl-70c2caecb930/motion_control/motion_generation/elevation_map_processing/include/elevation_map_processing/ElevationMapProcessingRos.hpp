/*
 * ElevationMapProcessingRos.hpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Aravind Elanjimattathil Vijayan
 *   Institute: ETH Zurich
 */

#pragma once

// ros.
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// geometry msgs.
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// grid-map.
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// stl.
#include <filters/filter_chain.h>
#include <ros/callback_queue.h>
#include <future>

// Eigen.
#include <Eigen/Core>
#include <boost/thread.hpp>

// boost.
#include <boost/thread.hpp>

namespace elevation_map_processing {

class ElevationMapProcessingRos {
 public:
  explicit ElevationMapProcessingRos(ros::NodeHandle& nodeHandle);
  ~ElevationMapProcessingRos();

 private:
  bool readParameters();
  void updateSubMap(const ros::TimerEvent& timerEvent);
  void runSubMapUpdateThread(std::future<void> futureObj);
  void updateSubMapOrigin(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void runSubMapOriginUpdateThread(std::future<void> futureObj);

  //! ros.
  ros::NodeHandle nodeHandle_;

  //! grid maps.
  grid_map::GridMap mapRaw_;
  grid_map::GridMap mapFiltered_;

  //! map service.
  std::string elevationMapServiceName_;
  ros::CallbackQueue subMapUpdateQueue_;
  ros::ServiceClient getSubMapServiceClient_;
  std::thread subMapUpdateThread_;
  ros::Duration subMapUpdatePeriod_;
  ros::Timer subMapUpdateTimer_;
  std::promise<void> promiseSubMapUpdateThread_;
  std::future<void> futureExitFlagSubMapUpdateThread_;

  //! grid-map filter.
  filters::FilterChain<grid_map::GridMap> subMapFilterChain_;

  //! Publisher
  ros::Publisher subMapPublisher_;
  boost::shared_mutex mutexSubMapOrigin_;

  //! sub map information.
  std::string poseFrameID_;
  Eigen::Vector3d subMapOriginInWorldFrame_;
  Eigen::Vector2d subMapSize_;

  //! Robot pose (origin of submap).
  ros::SubscribeOptions subMapOriginUpdateSubscriberOptions_;
  ros::Subscriber subMapOriginUpdateSubscriber_;
  std::thread subMapOriginUpdateThread_;
  ros::CallbackQueue subMapOriginUpdateQueue_;
  std::promise<void> exitSubMapOriginUpdateThread_;
  std::future<void> futureExitFlagSubMapOriginUpdateThread_;

  //! Transformation.
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  geometry_msgs::TransformStamped transformStamped_;
  Eigen::Isometry3d transformation_;

  //! False if updateSubMap has not yet been called.
  bool isFirstUpdate_;
};

}  // namespace elevation_map_processing
