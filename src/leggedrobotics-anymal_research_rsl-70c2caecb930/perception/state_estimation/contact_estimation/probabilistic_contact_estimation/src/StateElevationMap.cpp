/*
 * StateElevationMap.cpp
 *
 *  Created on: Sept 01, 2019
 *      Author: Fabian Jenelten
 */


// contact state estimation.
#include <probabilistic_contact_estimation/StateElevationMap.hpp>

// message logger.
#include <message_logger/message_logger.hpp>

// tinyxml tools.
#include <tinyxml_tools/tinyxml_tools.hpp>

// grid map.
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace contact_estimation {

StateElevationMap::StateElevationMap(anymal_model::AnymalModel* model, ros::NodeHandle& nodeHandle) :
    State(model),
    nodeHandle_(nodeHandle),
    subMapCopySubscriber_(),
    subMap_(),
    mutexSubMap_(),
    heightLayerName_("elevation_filled"),
    elevationMapTopicName_("/elevation_map_processing/sub_map"),
    distanceFeetOverElevationMap_(0.0),
    subMapAvailable_(false) {
}

StateElevationMap::~StateElevationMap() {
  nodeHandle_.shutdown();
}

void StateElevationMap::initializeSubscribers() {
  subMapCopySubscriber_.shutdown();
  subMapCopySubscriber_ = nodeHandle_.subscribe(elevationMapTopicName_, 1u, &StateElevationMap::copySubMap, this);
}

bool StateElevationMap::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle contactEstimationHandle = handle;

  if(!tinyxml_tools::getChildHandle(contactEstimationHandle, handle, "ProbabilisticContactEstimation")) {
    return false;
  }
  if(!tinyxml_tools::loadParameter(distanceFeetOverElevationMap_, contactEstimationHandle, "foot_center_over_elevation_map")) {
    return false;
  }

  // Sub map subscriber.
  elevationMapTopicName_ = nodeHandle_.param<std::string>("probabilistic_contact_estimation/sub_map_topic_name", elevationMapTopicName_);
  heightLayerName_ = nodeHandle_.param<std::string>("probabilistic_contact_estimation/height_layer_name", heightLayerName_);

  return true;
}

bool StateElevationMap::initialize(const double dt) {
  subMapAvailable_.store(false, std::memory_order_relaxed);
  initializeSubscribers();
  return State::initialize(dt);
}

double StateElevationMap::getHeight(const Eigen::Vector3d& positionWorldToLocationInWorldFrame) const {
  const grid_map::Position location(positionWorldToLocationInWorldFrame.x(), positionWorldToLocationInWorldFrame.y());
  double heightElevationMap;

  if(isHeightLayerAvailable() && getHeightFromElevationMapAtLocation(location, heightElevationMap)) {
    return heightElevationMap;
  }

  MELO_WARN_THROTTLE_STREAM(10.0, "Height layer of grid map was not available. Use blind terrain perception.");
  return State::getHeight(positionWorldToLocationInWorldFrame);
}

bool StateElevationMap::getHeightFromElevationMapAtLocation(const grid_map::Position& location, double& heightElevationMap) const {
  std::lock_guard<std::mutex> lockGridMap(mutexSubMap_);
  if (!subMap_.getLength().isZero() && subMap_.isInside(location)) {
    grid_map::Index index;
    if(subMap_.getIndex(location, index) && subMap_.isValid(index, heightLayerName_)) {
      heightElevationMap = subMap_.at(heightLayerName_, index);
      return true;
    }
  }
  MELO_WARN_THROTTLE_STREAM(10.0, "Failed to get height at position " << location << ".");
  return false;
}

bool StateElevationMap::updateExpectedTouchDownPosition(const std_utils::EnumArray<AD::ContactEnum, ContactState>& contactState) {
  if (!isHeightLayerAvailable()) {
    return State::updateExpectedTouchDownPosition(contactState);
  }
  // Update expected ground height at touch-down. Use elevation map only.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const double footHeight = getHeight(positionWorldToEndEffectorInWorldFrameVector_[contactEnum]) + distanceFeetOverElevationMap_;
    expectedGroundHeightInWorldFrame_[contactEnum].advance(footHeight);
  }

  return true;
}

void StateElevationMap::copySubMap(const grid_map_msgs::GridMap::ConstPtr &subMap) {
  std::vector<std::string> layers(1, heightLayerName_);

  try {
    std::lock_guard<std::mutex> lockGridMap(mutexSubMap_);
    if(!grid_map::GridMapRosConverter::fromMessage(*subMap, subMap_, layers, false, false)) {
      MELO_WARN_STREAM("Failed to convert grid map massage to grid map.");
      subMapAvailable_.store(false, std::memory_order_relaxed);
    } else {
      subMapAvailable_.store(true, std::memory_order_relaxed);
    }
  } catch(...) {
    MELO_WARN_STREAM("Function has thrown while converting grid map massage to grid map.");
    subMapAvailable_.store(false, std::memory_order_relaxed);
  }

  // Check if layers are available.
  if (!subMap_.exists(heightLayerName_)) {
    subMapAvailable_.store(false, std::memory_order_relaxed);
  }
}

bool StateElevationMap::isHeightLayerAvailable() const {
  return (subMapCopySubscriber_.getNumPublishers() > 0u && subMapAvailable_.load(std::memory_order_relaxed));
}

} /* namespace contact_estimation */
