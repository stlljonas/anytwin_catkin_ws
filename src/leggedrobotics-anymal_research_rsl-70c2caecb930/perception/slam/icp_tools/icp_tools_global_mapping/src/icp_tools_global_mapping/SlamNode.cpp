// header
#include "icp_tools_global_mapping/SlamNode.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace icp_tools_global_mapping {

SlamNode::SlamNode(ros::NodeHandle& nodeHandle) : SlamNodeT<PointCloudGlobalMapBuilder>(nodeHandle) {
  initSlamNode();
}

void SlamNode::initSlamNode() {
  // Read parameters from the ROS server.
  bool successReadingParameters = true;
  successReadingParameters &= readGeneralSlamParameters();
  successReadingParameters &= readLocalizationParameters();
  successReadingParameters &= readMappingParameters();

  // Read localization & mapping config files.
  successReadingParameters &= readConfigFiles();

  // Check if parameter loading was successful before going forward.
  if (!successReadingParameters) {
    ros::requestShutdown();
    return;
  }

  // Start the threads.
  initThreads();

  // Set up the node state (localization, mapping and pose publishing).
  setUpInitialNodeState();
}

bool SlamNode::processInputPointCloud(const PmStampedPointCloud& inputPointCloud) {
  // Get the initial guess for the robot pose in map.
  if (!getRobotToMapInitialGuess(currentInputPointCloudStamp_, tfRobotToMapInitialGuess_)) {
    MELO_WARN_STREAM("No initial guess available, localization is not executed.");
    return false;
  }

  // Consolidate map for localization.
  mapBuilder_.consolidateMapForLocalization(tfRobotToMapInitialGuess_);

  // Localization results containers.
  PmTf tfRobotToMap = tfRobotToMapInitialGuess_;
  PmMatrix covariance = PmMatrix::Zero(6, 6);

  // Check if the map is big enough for localization.
  activeMapCache_ = mapBuilder_.getMapInCache();
  const size_t mapSize = activeMapCache_.staticMapPointCloudSize_;
  if (mapSize < minMapSizeForLocalization_) {
    MELO_INFO_STREAM("Map is too small for localization (" << mapSize << "/" << minMapSizeForLocalization_ << " points).");

    // Add the input point cloud to the map if mapping is enabled.
    if (mappingIsEnabled_) {
      PointCloudGlobalMapBuilder::MappingInput mappingInput(inputPointCloud, tfRobotToMapInitialGuess_);
      mapBuilder_.stageForMapping(mappingInput);
    }

    localizationIsSuccessful_ = false;
  } else {
    // // At this point, the map is big enough for localization.

    // If enabled, run ICP localization.
    PmTf tfRobotToMap = tfRobotToMapInitialGuess_;
    PmMatrix covariance = PmMatrix::Zero(6, 6);
    if (localizationIsEnabled_) {
      localizationIsSuccessful_ = localize(inputPointCloud, tfRobotToMapInitialGuess_, tfRobotToMap, covariance);

      // Validate localization result and propagate flags within the system.
      validateLocalizationResult(localizationIsSuccessful_, tfRobotToMap, tfRobotToMapInitialGuess_, covariance);

      // Transform input point cloud to the map frame.
      matchedInputPointCloud_ = inputPointCloud;
      matchedInputPointCloud_.transform(tfRobotToMap);
    } else {
      localizationIsSuccessful_ = false;
    }
  }

  // Placeholder for maintaining current workflow.
  // TODO(ynava): In the future localizationIsSuccessful_ could be true if ICP fails but inertial/SE/TF or other contributing estimates are
  // consistent.
  if (localizationIsSuccessful_) {
    /* Mapping */
    // If the requirements are fulfilled, add the input point cloud to the map.
    if (mappingIsEnabled_) {
      PointCloudGlobalMapBuilder::MappingInput mappingInput(inputPointCloud, tfRobotToMap, icpOverlap_);
      mapBuilder_.stageForMapping(mappingInput);
    }

    // If enabled, update the robot pose in map using input from other localization modules via tf.
    if (updateRobotPoseInTime_) {
      updateRobotPoseInTime();
    } else {
      this->initialGuessRobotPoseInMap_.header = robotPoseInMap_.header;
      this->initialGuessRobotPoseInMap_.pose = robotPoseInMap_.pose.pose;
    }

    // Update the node output with the new robot pose.
    updateCorrectionTransformFromRobotPoseInMap();
  }

  /* ROS publishers */
  publishGeneralSlamTopics();

  return true;
}

}  // namespace icp_tools_global_mapping
