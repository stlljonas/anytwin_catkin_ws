// terrain_sensing.
#include <terrain_sensing/TerrainSensingRos.hpp>

// grid_map.
#include <grid_map_ros/grid_map_ros.hpp>

// ros.
#include <std_srvs/Empty.h>

// message logger.
#include <message_logger/message_logger.hpp>

namespace anymal_ctrl_dynamic_gaits_ros {
TerrainSensingRos::TerrainSensingRos(ros::NodeHandle& nodeHandle, loco::TerrainSensing& terrain)
    : nodeHandle_(nodeHandle), terrainSensing_(terrain), clearElevationMapAtInit_(false), subMap_() {}

TerrainSensingRos::~TerrainSensingRos() = default;

bool TerrainSensingRos::initialize() {
  // Reset promise signal to terminate thread
  exitSubMapCopyThread_ = std::promise<void>();

  // Get future object from the promise
  futureExitFlagSubMapCopyThread_ = exitSubMapCopyThread_.get_future();

  // Subscribe options for copying sub-map from the topic
  subMapCopySubscribeOptions_ = ros::SubscribeOptions::create<grid_map_msgs::GridMap>(
      "/elevation_map_processing/sub_map", 1, boost::bind(&TerrainSensingRos::copySubMap, this, _1), ros::VoidPtr(), &subMapCopyQueue_);
  subMapCopyQueue_.enable();
  subMapCopySubscriber_ = nodeHandle_.subscribe(subMapCopySubscribeOptions_);

  // Create thread that checks queue and copy sub-map
  subMapCopyThread_ = std::thread(&TerrainSensingRos::runCopyThread, this, std::move(futureExitFlagSubMapCopyThread_));

  if (clearElevationMapAtInit_) {
    clearElevationMap();
  }

  return true;
}

bool TerrainSensingRos::stop() {
  subMapCopyQueue_.clear();
  subMapCopyQueue_.disable();
  subMapCopySubscriber_.shutdown();

  // Send signal to terminate the thread
  exitSubMapCopyThread_.set_value();
  subMapCopyThread_.join();
  return true;
}

// copy sub-map into TerrainSensing
void TerrainSensingRos::copySubMap(const grid_map_msgs::GridMap::ConstPtr& subMap) {
  if (subMap == nullptr) {
    MELO_WARN_STREAM("[TerrainSensingRos::copySubMap] sub map pointer is null");
    return;
  }

  try {
    grid_map::GridMapRosConverter::fromMessage(*subMap, subMap_);
    terrainSensing_.setMap(subMap_);
  } catch (...) {
    MELO_WARN_STREAM("[TerrainSensingRos::copySubMap] Failed to get sub map");
  }
}

// Thread that check queue for available calls to copy sub-map
void TerrainSensingRos::runCopyThread(std::future<void> futureObj) {
  constexpr double timeout = 0.01;
  while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
    subMapCopyQueue_.callAvailable(ros::WallDuration(timeout));
  }
}

void TerrainSensingRos::clearElevationMap() {
  ros::ServiceClient elevationMapClient = nodeHandle_.serviceClient<std_srvs::Empty>("/elevation_mapping/clear_map");
  std_srvs::Empty elevationMapSrv;
  elevationMapClient.call(elevationMapSrv);
}

} /* namespace anymal_ctrl_dynamic_gaits_ros */
