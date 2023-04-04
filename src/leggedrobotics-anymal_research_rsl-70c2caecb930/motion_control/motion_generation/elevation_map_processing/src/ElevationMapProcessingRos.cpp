/*
 * ElevationMapProcessingRos.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Aravind Elanjimattathil Vijayan
 *   Institute: ETH Zurich
 */

// elevation_map_processing.
#include <elevation_map_processing/ElevationMapProcessingRos.hpp>

// grid-map.
#include <grid_map_msgs/GetGridMap.h>

// message logger.
#include <message_logger/message_logger.hpp>

using namespace message_logger::color;

namespace elevation_map_processing {
ElevationMapProcessingRos::ElevationMapProcessingRos(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      mapRaw_(),
      mapFiltered_(),
      elevationMapServiceName_("/elevation_mapping/get_submap"),
      subMapUpdateQueue_(),
      getSubMapServiceClient_(),
      subMapUpdateThread_(),
      subMapUpdatePeriod_(),
      subMapUpdateTimer_(),
      promiseSubMapUpdateThread_(),
      futureExitFlagSubMapUpdateThread_(promiseSubMapUpdateThread_.get_future()),
      subMapFilterChain_("grid_map::GridMap"),
      subMapPublisher_(),
      mutexSubMapOrigin_(),
      poseFrameID_(""),
      subMapOriginInWorldFrame_(Eigen::Vector3d::Zero()),
      subMapOriginUpdateSubscriberOptions_(),
      subMapOriginUpdateSubscriber_(),
      subMapOriginUpdateThread_(),
      subMapOriginUpdateQueue_(),
      exitSubMapOriginUpdateThread_(),
      futureExitFlagSubMapOriginUpdateThread_(exitSubMapOriginUpdateThread_.get_future()),
      tfBuffer_(),
      tfListener_(tfBuffer_),
      transformStamped_(),
      transformation_(Eigen::Isometry3d::Identity()),
      isFirstUpdate_(true) {
  // Load parameters from config file.
  if (!readParameters()) {
    nodeHandle.shutdown();
    MELO_FATAL_STREAM("[ElevationMapProcessingRos::ElevationMapProcessingRos] Failed to read parameters.");
    return;
  }

  // Set up filter chain (load filters as defined in default.yaml file).
  if (!subMapFilterChain_.configure("elevation_map_filters", nodeHandle_)) {
    nodeHandle.shutdown();
    MELO_FATAL_STREAM("ElevationMapProcessingRos::ElevationMapProcessingRos] Unable to configure filter chain.");
    return;
  }

  // ToDO: use standard subscriber

  // Advertise sub-map topic (publish the filtered map).
  subMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/elevation_map_processing/sub_map", 1, false);

  // Create elevation map service client (receive the elevation map).
  getSubMapServiceClient_ = nodeHandle_.serviceClient<grid_map_msgs::GetGridMap>(elevationMapServiceName_);

  // Create ROS timer for map update (call updateSubMap after each subMapUpdatePeriod_ seconds)
  if (!subMapUpdatePeriod_.isZero()) {
    ros::TimerOptions timerOptions(subMapUpdatePeriod_, boost::bind(&ElevationMapProcessingRos::updateSubMap, this, _1),
                                   &subMapUpdateQueue_, false, true);
    subMapUpdateTimer_ = nodeHandle_.createTimer(timerOptions);
  }

  // Set-up robot pose Subscriber (receive robot pose in odom).
  subMapOriginUpdateSubscriberOptions_ = ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(
      "/state_estimator/pose_in_odom", 1, boost::bind(&ElevationMapProcessingRos::updateSubMapOrigin, this, _1), ros::VoidPtr(),
      &subMapOriginUpdateQueue_);
  subMapOriginUpdateSubscriber_ = nodeHandle_.subscribe(subMapOriginUpdateSubscriberOptions_);

  // Create threads (continuously update queue).
  subMapUpdateThread_ = std::thread(&ElevationMapProcessingRos::runSubMapUpdateThread, this, std::move(futureExitFlagSubMapUpdateThread_));
  subMapOriginUpdateThread_ =
      std::thread(&ElevationMapProcessingRos::runSubMapOriginUpdateThread, this, std::move(futureExitFlagSubMapOriginUpdateThread_));
}

ElevationMapProcessingRos::~ElevationMapProcessingRos() {
  // Sub-map update thread clean-up.
  subMapUpdateQueue_.clear();
  subMapUpdateQueue_.disable();
  promiseSubMapUpdateThread_.set_value();  // fulfill promise (std::future_status will change from timeout to ready)
  subMapUpdateThread_.join();              // pause until function call finished

  // Sub-map origin update thread clean-up.
  subMapOriginUpdateQueue_.clear();
  subMapOriginUpdateQueue_.disable();
  exitSubMapOriginUpdateThread_.set_value();  // fulfill promise
  subMapOriginUpdateThread_.join();           // pause until function call finished

  subMapOriginUpdateSubscriber_.shutdown();
  nodeHandle_.shutdown();
}

bool ElevationMapProcessingRos::readParameters() {
  std::cout << magenta << "[ElevationMapProcessingRos] " << blue << "Load parameters for elevation mapping." << def << std::endl;

  // Service name.
  if (!nodeHandle_.getParam("sub_map/service_name", elevationMapServiceName_)) {
    MELO_WARN_STREAM("[ElevationMapProcessingRos::readParameters] Could not load sub-map service name.");
    return false;
  }

  // Update rate.
  double subMapUpdateRate;
  if (!nodeHandle_.getParam("sub_map/update_rate", subMapUpdateRate)) {
    MELO_WARN_STREAM("[ElevationMapProcessingRos::readParameters] Could not load sub-map update rate.");
    return false;
  }
  subMapUpdatePeriod_.fromSec((subMapUpdateRate > 0.0 ? 1.0 / subMapUpdateRate : 0.0));

  // Grid size X.
  if (!nodeHandle_.getParam("sub_map/sub_map_length_x", subMapSize_.x())) {
    MELO_WARN_STREAM("[ElevationMapProcessingRos::readParameters] Could not load sub-map length x.");
    return false;
  }

  // Grid size Y.
  if (!nodeHandle_.getParam("sub_map/sub_map_length_y", subMapSize_.y())) {
    MELO_WARN_STREAM("[ElevationMapProcessingRos::readParameters] Could not load sub-map length y.");
    return false;
  }

  return true;
}
void ElevationMapProcessingRos::updateSubMap(const ros::TimerEvent& timerEvent) {
  bool didReceiveElevationMap = true;

  // Wait for pose measurement.
  if (poseFrameID_.empty()) {
    return;
  }

  // Check if we need to transform grid map.
  const bool transformGridMap = (!isFirstUpdate_ && mapRaw_.getFrameId() != poseFrameID_);

  /**************************************
   * Compute transformation map to odom *
   **************************************/
  if (didReceiveElevationMap && transformGridMap) {
    try {
      transformStamped_ = tfBuffer_.lookupTransform(poseFrameID_, mapRaw_.getFrameId(), ros::Time(0));
    } catch (...) {
      MELO_WARN_THROTTLE_STREAM(0.5, "[ElevationMapProcessingRos::updateSubMap] Failed to get transformation "
                                         << mapRaw_.getFrameId() << " (map) to " << poseFrameID_ << ".");
      return;
    }

    // Extract translation.
    transformation_.translation().x() = transformStamped_.transform.translation.x;
    transformation_.translation().y() = transformStamped_.transform.translation.y;
    transformation_.translation().z() = transformStamped_.transform.translation.z;

    // Extract rotation.
    Eigen::Quaterniond rotationQuaternion(transformStamped_.transform.rotation.w, transformStamped_.transform.rotation.x,
                                          transformStamped_.transform.rotation.y, transformStamped_.transform.rotation.z);
    transformation_.linear() = rotationQuaternion.toRotationMatrix();
  }
  /**************************************/

  /*********************
   * Configure sub-map *
   *********************/
  grid_map_msgs::GetGridMap subMapService;
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexSubMapOrigin_);
    const Eigen::Vector3d positionMapToOriginInMapFrame =
        (transformGridMap ? transformation_.inverse() * subMapOriginInWorldFrame_ : subMapOriginInWorldFrame_);
    subMapService.request.position_x = positionMapToOriginInMapFrame.x();
    subMapService.request.position_y = positionMapToOriginInMapFrame.y();
  }
  subMapService.request.length_x = subMapSize_.x();
  subMapService.request.length_y = subMapSize_.y();
  /*********************/

  /***************
   * Get sub-map *
   ***************/
  if (didReceiveElevationMap && !getSubMapServiceClient_.call(subMapService)) {
    MELO_WARN_THROTTLE_STREAM(1.0, "[ElevationMapProcessingRos::updateSubMap] Sub-Map service call failed.");
    didReceiveElevationMap = false;
  }

  if (didReceiveElevationMap && !grid_map::GridMapRosConverter::fromMessage(subMapService.response.map, mapRaw_)) {
    MELO_WARN_THROTTLE_STREAM(1.0, "[ElevationMapProcessingRos::updateSubMap] Sub map conversion failed.");
    didReceiveElevationMap = false;
  }

  // For the first time, we need to call updateSubMap twice since the map frame is unknown at the first update.
  if (didReceiveElevationMap && isFirstUpdate_) {
    isFirstUpdate_ = false;
    return updateSubMap(timerEvent);
  }
  /***************/

  /**************************************************
   * Transform to odom frame and apply filter chain *
   **************************************************/
  if (didReceiveElevationMap) {
    if (transformGridMap) {
      if (!subMapFilterChain_.update(mapRaw_.getTransformedMap(transformation_, "elevation", poseFrameID_, 0.0), mapFiltered_)) {
        MELO_WARN_THROTTLE_STREAM(
            1.0, "[ElevationMapProcessingRos::updateSubMap] Could not update the sub-map filter chain for transformed map!");
        didReceiveElevationMap = false;
      }
    } else {
      if (!subMapFilterChain_.update(mapRaw_, mapFiltered_)) {
        MELO_WARN_THROTTLE_STREAM(1.0, "[ElevationMapProcessingRos::updateSubMap] Could not update the sub-map filter chain!");
        didReceiveElevationMap = false;
      }
    }
  }
  /**************************************************/

  // In case something went wrong: Create dummy map.
  if (!didReceiveElevationMap) {
    mapRaw_.setGeometry(subMapSize_, 0.02, subMapOriginInWorldFrame_.head<2>());
    mapFiltered_.setGeometry(subMapSize_, 0.02, subMapOriginInWorldFrame_.head<2>());
    mapRaw_.setFrameId(poseFrameID_);
    mapFiltered_.setFrameId(poseFrameID_);
  }

  /****************************
   * Publish filtered sub-map *
   ***************************/
  if (subMapPublisher_.getNumSubscribers() > 0u) {
    grid_map_msgs::GridMap msgMapFiltered;
    grid_map::GridMapRosConverter::toMessage(mapFiltered_, msgMapFiltered);
    subMapPublisher_.publish(msgMapFiltered);
  }
  /***************************/
}

void ElevationMapProcessingRos::updateSubMapOrigin(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (msg == nullptr) {
    MELO_WARN_STREAM("[ElevationMapProcessingRos::updateSubMapOrigin] msg is null");
    return;
  }

  // Copy robot position in odom.
  try {
    boost::unique_lock<boost::shared_mutex> lock(mutexSubMapOrigin_);
    poseFrameID_ = msg->header.frame_id;
    subMapOriginInWorldFrame_.x() = msg->pose.pose.position.x;
    subMapOriginInWorldFrame_.y() = msg->pose.pose.position.y;
    subMapOriginInWorldFrame_.z() = msg->pose.pose.position.z;
  } catch (...) {
    MELO_WARN_STREAM("[ElevationMapProcessingRos::updateSubMapOrigin] Failed to read pose");
  }
}

void ElevationMapProcessingRos::runSubMapUpdateThread(std::future<void> futureObj) {
  // Wait for 1ms.
  while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
    subMapUpdateQueue_.callAvailable(ros::WallDuration(0.01));
  }
}

void ElevationMapProcessingRos::runSubMapOriginUpdateThread(std::future<void> futureObj) {
  // Wait for 1ms.
  while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    subMapOriginUpdateQueue_.callAvailable(ros::WallDuration(0.01));
}

}  // namespace elevation_map_processing
