
// header
#include "icp_tools_global_mapping/PointCloudGlobalMapBuilder.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// param io
#include <param_io/get_param.hpp>

// slam point cloud mapping
#include <slam_point_cloud_mapping/utils.hpp>

namespace icp_tools_global_mapping {

PointCloudGlobalMapBuilder::PointCloudGlobalMapBuilder() : parameters_(Parameters()) {}

PointCloudGlobalMapBuilder::PointCloudGlobalMapBuilder(const Parameters parameters) : parameters_(std::move(parameters)) {}

size_t PointCloudGlobalMapBuilder::getNumberOfMaps() const {
  return 1;
}

bool PointCloudGlobalMapBuilder::isLoopDetectionEnabled() const {
  return false;
}

void PointCloudGlobalMapBuilder::consolidateMapForLocalization(const PmTf& /*tfRobotToMap*/) {
  std::lock_guard<std::mutex> lockMap(mapMutex_);
  dynamicPointsDetector_.splitPointCloud(map_, mapCache_.dynamicMapPointCloudInMapFrame_, mapCache_.staticMapPointCloudInMapFrame_);

  mapCache_.staticMapPointCloudSize_ = mapCache_.staticMapPointCloudInMapFrame_.getSize();
  mapCache_.dynamicMapPointCloudSize_ = mapCache_.dynamicMapPointCloudInMapFrame_.getSize();
}

void PointCloudGlobalMapBuilder::processDataInMappingQueue() {
  // Create a copy of the map to modify, such that the localization can continue and
  // we only need to lock it later when copying back.
  // This operation is very fast, 100'000 points take roughly 1ms.
  PmStampedPointCloud map = map_;

  // Get the mapping input.
  std::vector<MappingInput> mappingQueue;
  {
    std::lock_guard<std::mutex> lockMappingInput(mappingQueueMutex_);
    mappingQueue = mappingQueue_;
    mappingQueue_.clear();
  }

  // Loop through all the point clouds in the input vector.
  for (auto& data : mappingQueue) {
    if (data.icpOverlap_ > parameters_.maxOverlapForMapping_) {
      continue;
    }

    // Initialize dynamic points descriptors of input point cloud.
    dynamicPointsDetector_.initializeDescriptors(data.pointCloudInRobotFrame_);

    // Extend map.
    if (!extendMap(data.pointCloudInRobotFrame_, data.tfRobotToMap_, map)) {
      MELO_ERROR_STREAM("Map could not be extended.");
      operationalStatus_ = OperationalStatusCode::ERROR;
      continue;
    }

    operationalStatus_ = OperationalStatusCode::OK;
  }

  // Filter the map.
  // This is typically where a lot of the computation time is spent.
  // If this was done in the loop, the mapper might not be able to catch up with the localization anymore.
  {
    std::lock_guard<std::mutex> lockConfig(mapFiltersMutex_);
    if (!map.filter(mapFilters_)) {
      MELO_ERROR_STREAM("Could not filter map after after adding point cloud.");
      return;
    }
  }

  // Update the map with our extended copy.
  {
    // Do not allow the ROS thread to read from the map during the update.
    std::lock_guard<std::mutex> lockMap(mapMutex_);
    map_ = map;
  }
}

bool PointCloudGlobalMapBuilder::extendMap(const PmStampedPointCloud& pointCloudInRobotFrame, const PmTf& tfRobotToMap,
                                           PmStampedPointCloud& map) {
  // Compute the probability of a point being dynamic
  dynamicPointsDetector_.updateDynamicProbability(pointCloudInRobotFrame, tfRobotToMap, parameters_.maxSensorRange_, map);

  // Transform the point cloud to the map frame.
  PmStampedPointCloud pointCloudInMapFrame = pointCloudInRobotFrame;
  if (!pointCloudInMapFrame.transform(tfRobotToMap)) {
    MELO_ERROR_STREAM("Point cloud could not be transformed to map frame.");
    return false;
  }

  // If the map is empty, use the point cloud in map frame as a seed.
  if (map.isEmpty()) {
    map = pointCloudInMapFrame;
    MELO_DEBUG_STREAM("Map initialized.");
    return true;
  }

  // Split the input point cloud by the overlap with the map.
  PmStampedPointCloud overlappingPoints;
  PmStampedPointCloud nonOverlappingPoints;
  if (!map.splitByOverlap(pointCloudInMapFrame, parameters_.maxDistanceOverlappingPoints_, overlappingPoints, nonOverlappingPoints)) {
    MELO_ERROR_STREAM("Overlap could not be found.");
    return false;
  }

  // Only add the non-overlapping (new) points to the map.
  if (!map.add(nonOverlappingPoints)) {
    MELO_ERROR_STREAM("Point cloud could not be added to map.");
    return false;
  }

  // Remove dynamic points from the map if requested.
  if (parameters_.removeDynamicPointsAfterExtendingMap_) {
    if (!dynamicPointsDetector_.removeDynamicPoints(map)) {
      MELO_ERROR_STREAM("Failed to remove dynamic points from the map.");
      return false;
    }
  }

  return true;
}

bool PointCloudGlobalMapBuilder::loadMapFromFilesystem(const std::string& filePath, const ros::Time& stamp) {
  std::lock_guard<std::mutex> lockMappingIsOngoing(mappingIsOngoingMutex_);
  std::lock_guard<std::mutex> lockMap(mapMutex_);

  // Load the map from the file.
  if (!map_.fromFile(filePath, stamp, coordinateFrames_.mapFrameId())) {
    MELO_ERROR_STREAM("Failed to load map from file '" << filePath << "'.");
    return false;
  }
  MELO_INFO_STREAM("Loaded map with " << map_.getSize() << " points.");

  map_.setDescriptorFromDescriptor(POINT_CLOUD_DESCRIPTOR_NAME_PROB_STATIC, POINT_CLOUD_DESCRIPTOR_NAME_CURVATURE);
  map_.addOneDimensionalDescriptor(POINT_CLOUD_DESCRIPTOR_NAME_DEBUG, 0.0);

  // Filter the map.
  {
    std::lock_guard<std::mutex> lockConfig(mapFiltersMutex_);
    if (!map_.filter(mapFilters_)) {
      MELO_ERROR_STREAM("Could not filter map after loading from file.");
      return false;
    }
  }

  MELO_INFO_STREAM("Map contains " << map_.getSize() << " points after filtering.");
  return true;
}

bool PointCloudGlobalMapBuilder::saveMapToFilesystem(const std::string& absolutePath) const {
  // Save the map to the file.
  PmStampedPointCloud map;
  {
    std::lock_guard<std::mutex> lockMap(mapMutex_);
    map = map_;
  }

  map.setDescriptorFromDescriptor(POINT_CLOUD_DESCRIPTOR_NAME_CURVATURE, POINT_CLOUD_DESCRIPTOR_NAME_PROB_STATIC);
  map.toFile(absolutePath);
  return true;
}

void PointCloudGlobalMapBuilder::clearMap(const ros::Time& stamp) {
  // Clear and re-initialize the map.
  std::lock_guard<std::mutex> lockMappingIsOngoing(mappingIsOngoingMutex_);
  std::lock_guard<std::mutex> lockMap(mapMutex_);

  // Map cache.
  mapCache_.clear();

  // Map point cloud.
  map_.clear();
  map_.header_.stamp = stamp;
  map_.header_.frame_id = coordinateFrames_.mapFrameId();

  MELO_INFO_STREAM("Map has been cleared.");
}

bool PointCloudGlobalMapBuilder::setUpFromRos(ros::NodeHandle& nodeHandle) {
  bool success = true;

  // Get global map builder parameters.
  PointCloudGlobalMapBuilder::Parameters mapBuilderParameters;
  success &= param_io::getParam(nodeHandle, "mapping/max_overlap", mapBuilderParameters.maxOverlapForMapping_);
  success &= param_io::getParam(nodeHandle, "mapping/max_distance_overlapping_points", mapBuilderParameters.maxDistanceOverlappingPoints_);
  success &= param_io::getParam(nodeHandle, "common/max_sensor_range", mapBuilderParameters.maxSensorRange_);
  success &= param_io::getParam(nodeHandle, "mapping/remove_dynamic_points_after_extending_map",
                                mapBuilderParameters.removeDynamicPointsAfterExtendingMap_);
  setParameters(mapBuilderParameters);

  // Get parameters and init dynamic points detector.
  success &= setUpDynamicPointsDetectorFromRos(nodeHandle);

  return success;
}

}  // namespace icp_tools_global_mapping
