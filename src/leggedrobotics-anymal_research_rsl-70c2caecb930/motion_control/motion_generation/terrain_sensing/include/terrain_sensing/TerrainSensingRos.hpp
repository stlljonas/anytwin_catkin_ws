#pragma once

// terrain sensing.
#include <terrain_sensing/TerrainSensing.hpp>

// grid map.
#include <grid_map_msgs/GridMap.h>

// thread.
#include <future>
#include <thread>

// ros.
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace anymal_ctrl_dynamic_gaits_ros {
class TerrainSensingRos {
 public:
  TerrainSensingRos(ros::NodeHandle& nodeHandle, loco::TerrainSensing& terrain);
  virtual ~TerrainSensingRos();
  virtual bool initialize();
  virtual bool stop();

 private:
  // Thread that check queue for available calls to copy sub-map.
  void runCopyThread(std::future<void> futureObj);

  // Copy sub-map into TerrainSensing.
  void copySubMap(const grid_map_msgs::GridMapConstPtr& subMap);

  // Process the sub-map.
  ros::NodeHandle& nodeHandle_;
  loco::TerrainSensing& terrainSensing_;

  //! Calls service to clear elevation map.
  void clearElevationMap();

  // Sub-map copy.
  std::promise<void> exitSubMapCopyThread_;
  std::future<void> futureExitFlagSubMapCopyThread_;
  std::thread subMapCopyThread_;
  ros::SubscribeOptions subMapCopySubscribeOptions_;
  ros::CallbackQueue subMapCopyQueue_;
  ros::Subscriber subMapCopySubscriber_;

  //! If true, elevation map is cleared during initialization.
  bool clearElevationMapAtInit_;

  //! Local copy of sub map.
  grid_map::GridMap subMap_;
};
}  // namespace anymal_ctrl_dynamic_gaits_ros
