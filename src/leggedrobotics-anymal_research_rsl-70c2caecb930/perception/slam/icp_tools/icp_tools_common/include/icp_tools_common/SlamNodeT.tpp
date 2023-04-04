
// std
#include <mutex>
#include <thread>

// stopwatch
#include <stopwatch/Stopwatch.hpp>

// message logger
#include <message_logger/message_logger.hpp>

// slam point cloud mapping
#include <slam_point_cloud_mapping/macros.hpp>
#include <slam_point_cloud_mapping/utils.hpp>

// pointmatcher_ros
#include <pointmatcher_ros/helper_functions.h>
#include <pointmatcher_ros/transform.h>

// tf conversions
#include <tf_conversions/tf_eigen.h>

// kindr ros
#include <kindr_ros/kindr_ros.hpp>

// slam common ros
#include <slam_common_ros/ros_transport.hpp>

// geometry msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

// tf
#include <tf/transform_datatypes.h>

// slam common msgs
#include <slam_common_msgs/NodeState.h>

namespace icp_tools_common {

template <typename MapBuilderType>
SlamNodeT<MapBuilderType>::SlamNodeT(ros::NodeHandle& nodeHandle)
    : SlamInterface<MapBuilderType>(nodeHandle),
      icpOverlap_(0.0),
      localizationOperationalStatus_(OperationalStatusCode::IDLE),
      inputPointCloudAge_(0.0),
      inputPointCloudSize_(0),
      timeBetweenInputPointClouds_(0.0),
      timeUsedForLocalization_(0.0),
      consecutiveLocalizationFails_(0) {
  setUpGeneralPublishers();
  setUpGeneralSubscribers();
  setUpGeneralServices();
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::readGeneralSlamParameters() {
  bool success = true;

  success &= param_io::getParam(this->nodeHandle_, "common/max_sensor_range", maxSensorRange_);
  success &= param_io::getParam(this->nodeHandle_, "common/max_sensor_range_scaling_to_cut_map", maxSensorRangeScalingToCutMap_);

  // Read the configuration files.
  success &= param_io::getParam(this->nodeHandle_, "config_files_folder", configFilesFolder_);

  return success;
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::readLocalizationParameters() {
  bool success = true;

  // Tf.
  success &= param_io::getParam(this->nodeHandle_, "localization/initial_guess/use_tf", useTf_);
  success &= param_io::getParam(this->nodeHandle_, "localization/initial_guess/update_tf_duration", updateTfDuration_);
  success &= param_io::getParam(this->nodeHandle_, "localization/initial_guess/wait_for_tf_duration", waitForTfDuration_);

  // Icp.
  success &= param_io::getParam(this->nodeHandle_, "localization/icp/min_map_size", minMapSizeForLocalization_);
  success &= param_io::getParam(this->nodeHandle_, "localization/icp/min_input_point_cloud_size", minInputPointCloudSizeForLocalization_);
  success &= param_io::getParam(this->nodeHandle_, "localization/icp/min_overlap", minOverlapForLocalization_);

  // Gravity alignment and propagation.
  success &= param_io::getParam(this->nodeHandle_, "localization/result/enforce_map_gravity_alignment", enforceMapGravityAlignment_);
  success &= param_io::getParam(this->nodeHandle_, "localization/result/update_time_using_odometry", updateRobotPoseInTime_);

  // Robustness.
  success &= param_io::getParam(this->nodeHandle_, "robustness/consecutive_localization_fails_before_reset",
                                consecutiveLocalizationFailsBeforeReset_);
  success &= param_io::getParam(this->nodeHandle_, "robustness/localization_correction_position_threshold",
                                localizationCorrectionPositionThreshold_);
  success &= param_io::getParam(this->nodeHandle_, "robustness/localization_correction_rotation_threshold",
                                localizationCorrectionRotationThreshold_);

  // Read the configuration files.
  success &= param_io::getParam(this->nodeHandle_, "localization/icp/config_file_name", icpConfigFileName_);

  return success;
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::readMappingParameters() {
  bool success = true;

  // Map builder.
  success &= this->mapBuilder_.setUpFromRos(this->nodeHandle_);

  // Map filters.
  success &= param_io::getParam(this->nodeHandle_, "mapping/filters_file_name", mapFiltersFileName_);

  return success;
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::readConfigFiles() {
  bool success = true;
  MELO_INFO_STREAM("Reading the configuration files ...");
  success &= slam_point_cloud_mapping::readIcpConfigFile(icp_, configFilesFolder_ + "/" + icpConfigFileName_);
  success &= this->mapBuilder_.readMapFiltersConfigFile(configFilesFolder_ + "/" + mapFiltersFileName_);
  return success;
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::readConfigFilesCb(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  // Read the configuration files.
  return readConfigFiles();
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::setUpLocalizationSubscribers() {
  slam_common_ros::subscribe(this->nodeHandle_, inputPointCloudSubscriber_, "input_point_cloud",
                             &SlamNodeT<MapBuilderType>::inputPointCloudCb, this);
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::shutdownLocalizationSubscribers() {
  inputPointCloudSubscriber_.shutdown();
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::resetLocalizationFlags() {
  // Status.
  localizationOperationalStatus_ = OperationalStatusCode::IDLE;
  localizationIsSuccessful_ = false;

  // Metrics.
  this->timeUsedForLocalization_ = 0.0;
  icpOverlap_ = 0.0;
  consecutiveLocalizationFails_ = 0;
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::inputPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& pointCloudRos) {
  MELO_DEBUG_STREAM("Received new input point cloud.");

  // Update state variables.
  lastInputPointCloudStamp_ = currentInputPointCloudStamp_;
  currentInputPointCloudStamp_ = pointCloudRos->header.stamp;
  if (!lastInputPointCloudStamp_.isZero()) {
    timeBetweenInputPointClouds_ = (currentInputPointCloudStamp_ - lastInputPointCloudStamp_).toSec();
  }
  inputPointCloudAge_ = (ros::Time::now() - currentInputPointCloudStamp_).toSec();
  inputPointCloudSize_ = pointCloudRos->height * pointCloudRos->width;

  // Check if the input point cloud fulfills basic requirements before processing it.
  // Input data frame ID.
  if (pointCloudRos->header.frame_id != this->coordinateFrames_.robotFrameId()) {
    MELO_ERROR_STREAM("Expected input point cloud to be in the robot frame: " << pointCloudRos->header.frame_id
                                                                              << " != " << this->coordinateFrames_.robotFrameId()
                                                                              << ", localization is not executed.");
    return;
  }

  // Number of points.
  if (inputPointCloudSize_ == 0) {
    MELO_INFO_STREAM("Input point cloud is empty, localization is not executed.");
    return;
  } else {
    MELO_DEBUG_STREAM("Input point cloud has " << inputPointCloudSize_ << " points.");
  }

  // Convert the input point cloud to a pointmatcher type.
  PmStampedPointCloud pointCloud = PmStampedPointCloud::FromRosMsg(*pointCloudRos);

  // Process input data.
  processInputPointCloud(pointCloud);
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::getRobotToMapInitialGuess(const ros::Time& desiredTime, PmTf& tfRobotToMap) {
  MELO_DEBUG_STREAM("Getting new initial guess for map to robot transformation.");

  // Get the current pose of the robot in map.
  bool robotPoseIsEmpty = false;
  {
    std::lock_guard<std::mutex> lock(this->robotPoseInMapMutex_);
    robotPoseIsEmpty = this->initialGuessRobotPoseInMap_.header.stamp.isZero();
  }

  // Compute time passed after enabling localization.
  const bool tfIsUpdated = (desiredTime - this->timeOfTogglingPublishPose_).toSec() > updateTfDuration_;

  MELO_DEBUG_STREAM("Robot pose is " << (robotPoseIsEmpty ? "empty." : "valid."));
  MELO_DEBUG_STREAM("Tf is " << (tfIsUpdated ? "updated." : "not updated."));

  // Initial guess Tf lookup.
  //  This block of code runs if:
  //    * No explicit initial guess has been received yet, or
  //    * The user wants the SLAM system to look for an initial guess in the Tf, every iteration.
  bool initialGuessReceivedFromTf = false;
  if (robotPoseIsEmpty || (useTf_ && this->publishPoseIsEnabled_ && tfIsUpdated)) {
    // Try to get tf at desired point in time.
    try {
      const geometry_msgs::TransformStamped tfMsg = this->tfBuffer_.lookupTransform(
          this->coordinateFrames_.mapFrameId(), this->coordinateFrames_.robotFrameId(), desiredTime, ros::Duration(waitForTfDuration_));
      MELO_DEBUG_STREAM("Using tf as initial guess.");
      tfRobotToMap.fromRosTfMsg(tfMsg);
      initialGuessReceivedFromTf = true;
    } catch (const tf::TransformException& exception) {
      MELO_WARN_STREAM("Caught a tf exception while getting new initial guess "
                       << "for map to robot transformation: '" << exception.what() << "'.");
    }
  }

  // Early exit if there is no initial guess and Tf lookup was not successful. Essentially, the system has not been bootstrapped.
  if (robotPoseIsEmpty && !initialGuessReceivedFromTf) {
    // At this point, the robot pose is empty (no initial guess or prior localization), and the use of tf has failed.
    MELO_WARN_STREAM("No initial guess received nor prior localization result available.");
    return false;
  }

  // If Tf lookup failed or is disabled, the SLAM system will use the result from the last localization step run.
  // This block will only run if an explicit initial guess has already been received.
  MELO_DEBUG_STREAM("Using last robot pose as initial guess.");
  if (!useTf_ || !initialGuessReceivedFromTf) {
    std::lock_guard<std::mutex> lock(this->robotPoseInMapMutex_);
    tfRobotToMap = PointMatcher_ros::poseToPmTf(this->initialGuessRobotPoseInMap_, this->coordinateFrames_.robotFrameId());
  }

  // Store initial guess.
  this->initialGuessRobotPoseInMap_ = pmTfToPose(tfRobotToMap);
  this->initialGuessIsReceived_ = true;

  return true;
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::localize(const PmStampedPointCloud& inputPointCloud, const PmTf& tfRobotToMapInitialGuess,
                                         PmTf& tfRobotToMap, PmMatrix& covariance) {
  // Check if the input point cloud is too small.
  if (inputPointCloud.getSize() < minInputPointCloudSizeForLocalization_) {
    MELO_INFO_STREAM("Input point cloud is too small for localization (" << inputPointCloud.getSize() << "/"
                                                                         << minInputPointCloudSizeForLocalization_ << " points).");
    return false;
  }

  // Start the stop watch for measuring localization execution time.
  stopwatch::Stopwatch stopwatch("Localization");
  stopwatch.start();

  // Process map point cloud before running localization.
  PmStampedPointCloud processedMapPointCloud =
      processMapForLocalization(activeMapCache_.staticMapPointCloudInMapFrame_, tfRobotToMapInitialGuess);

  // Run ICP registration.
  const bool localizationIsSuccessful = match(inputPointCloud, processedMapPointCloud, tfRobotToMapInitialGuess, tfRobotToMap, covariance);

  if (localizationIsSuccessful) {
    // If enabled, enforce gravity alignment of the map frame.
    if (enforceMapGravityAlignment_) {
      alignPoseInMapWithGravity(tfRobotToMap);
    }

    // Localization was successful, convert the resulting transformation to a pose.
    const geometry_msgs::PoseStamped robotPoseInMap = pmTfToPose(tfRobotToMap);
    {
      std::lock_guard<std::mutex> lock(this->robotPoseInMapMutex_);
      this->robotPoseInMap_.header = robotPoseInMap.header;
      this->robotPoseInMap_.pose.pose = robotPoseInMap.pose;
      for (unsigned int i = 0; i < 36; i++) {
        this->robotPoseInMap_.pose.covariance[i] = covariance(i / 6, i % 6);
      }
    }

    localizationOperationalStatus_ = OperationalStatusCode::OK;
  } else {
    localizationOperationalStatus_ = OperationalStatusCode::ERROR;
  }

  stopwatch.stop();
  this->timeUsedForLocalization_ = stopwatch.getStatistics().getLastMeasurement();

  return localizationIsSuccessful;
}

template <typename MapBuilderType>
PmStampedPointCloud SlamNodeT<MapBuilderType>::processMapForLocalization(const PmStampedPointCloud& mapPointCloud,
                                                                         const PmTf& tfRobotToMapInitialGuess) {
  PmStampedPointCloud processedStaticMap = mapPointCloud;

  // Distance filter.
  processedStaticMap.transform(tfRobotToMapInitialGuess.inverse());
  processedStaticMap.filterByDistance(maxSensorRangeScalingToCutMap_ * maxSensorRange_, true);
  processedStaticMap.transform(tfRobotToMapInitialGuess);

  // TODO(ynava) Implement filtering with bounding boxes in arbitrary frames (coordinates given by user or dynamic object detection)

  return processedStaticMap;
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::match(const PmStampedPointCloud& inputPointCloud, const PmStampedPointCloud& mapPointCloud,
                                      const PmTf& tfRobotToMapInitialGuess, PmTf& tfRobotToMap, PmMatrix& covariance) {
  // Run the ICP sequence.
  PmTfParameters correctedTfRobotToMapParameters;
  try {
    correctedTfRobotToMapParameters =
        icp_.compute(inputPointCloud.dataPoints_, mapPointCloud.dataPoints_, tfRobotToMapInitialGuess.parameters_);
  } catch (const std::exception& exception) {
    MELO_ERROR_STREAM("Caught an exception while running ICP (are your point clouds overlapping?): " << exception.what());
    return false;
  }

  // Check if the overlap is sufficient for localization.
  icpOverlap_ = icp_.errorMinimizer->getOverlap();
  MELO_DEBUG_STREAM("ICP overlap: " << icpOverlap_);
  if (icpOverlap_ < minOverlapForLocalization_) {
    MELO_ERROR_STREAM("ICP overlap too small (" << icpOverlap_ << "/" << minOverlapForLocalization_ << "), ignoring result.");
    return false;
  }

  // Get covariance from error minimizer.
  covariance = icp_.errorMinimizer->getCovariance();

  MELO_DEBUG_STREAM("Initial guess was corrected by:" << std::endl
                                                      << tfRobotToMapInitialGuess.parameters_.inverse() * correctedTfRobotToMapParameters);

  tfRobotToMap.stamp_ = inputPointCloud.header_.stamp;
  tfRobotToMap.parameters_ = correctedTfRobotToMapParameters;
  return true;
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::alignPoseInMapWithGravity(PmTf& tfRobotToMap) const {
  // Get the transformation from odometry to robot.
  geometry_msgs::TransformStamped tfMsg;
  try {
    tfMsg = this->tfBuffer_.lookupTransform(this->coordinateFrames_.robotFrameId(), this->coordinateFrames_.odometryFrameId(),
                                            tfRobotToMap.stamp_, ros::Duration(waitForTfDuration_));
  } catch (const tf::TransformException& exception) {
    MELO_ERROR_STREAM("Caught a tf exception while aligning map frame with gravity: " << exception.what());
    return false;
  }
  tf::StampedTransform tf;
  tf::transformStampedMsgToTF(tfMsg, tf);
  HomTransformQuat T_RO;
  kindr_ros::convertFromRosTf(tf, T_RO);

  // Convert the robot to map transformation to a kindr object.
  HomTransformQuat T_MR;
  geometry_msgs::PoseStamped robotPoseInMap = pmTfToPose(tfRobotToMap);
  kindr_ros::convertFromRosGeometryMsg(robotPoseInMap.pose, T_MR);

  // Compute the odometry to map transformation.
  HomTransformQuat T_MO = T_MR * T_RO;
  T_MO.getRotation().fix();

  // Compute the current gravity direction.
  const Eigen::Vector3d O_gravDir = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d M_gravDir = T_MO.getRotation().rotate(O_gravDir).normalized();

  // Compute the alignment rotation.
  HomTransformQuat::Rotation A_gAlign;
  A_gAlign.setFromVectors(M_gravDir, O_gravDir);

  // Change the orientation of the robot frame such that gravity alignment is
  HomTransformQuat T_MR_gAligned;
  T_MR_gAligned.getPosition() = T_MR.getPosition();
  T_MR_gAligned.getRotation() = A_gAlign * T_MR.getRotation();
  T_MR_gAligned.getRotation().fix();

  // Sanity output.
  MELO_DEBUG_STREAM("Gravity direction in map frame before alignment:" << std::endl << M_gravDir.transpose());
  MELO_DEBUG_STREAM("Gravity direction in map frame after alignment:"
                    << std::endl
                    << T_MR_gAligned.getRotation().rotate(T_RO.getRotation().rotate(O_gravDir)).normalized().transpose());

  // Convert back to pointmatcher tf.
  geometry_msgs::PoseStamped gAlignedRobotPoseInMap;
  gAlignedRobotPoseInMap.header.stamp = tfRobotToMap.stamp_;
  gAlignedRobotPoseInMap.header.frame_id = tfRobotToMap.targetFrameId_;
  kindr_ros::convertToRosGeometryMsg(T_MR_gAligned, gAlignedRobotPoseInMap.pose);
  tfRobotToMap = PointMatcher_ros::poseToPmTf(gAlignedRobotPoseInMap, tfRobotToMap.sourceFrameId_);

  return true;
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::validateLocalizationResult(const bool localizationIsSuccessful, const PmTf& tfRobotToMap,
                                                           const PmTf& tfRobotToMapInitialGuess, const PmMatrix& covariance) {
  // Compute the norm of the localization correction.
  const PmTfParameters correctionTransform = tfRobotToMapInitialGuess.parameters_.inverse() * tfRobotToMap.parameters_;
  const Eigen::Vector3f positionCorrection = correctionTransform.block<3, 1>(0, 3);
  const Eigen::Quaternionf rotationCorrection(correctionTransform.block<3, 3>(0, 0));
  float positionCorrectionNorm = positionCorrection.norm();
  float rotationCorrectionNorm = rotationCorrection.coeffs().block<3, 1>(0, 0).norm();

  // Determine localization status.
  if (localizationIsSuccessful) {
    bool localizationResultIsReasonable = true;
    if (localizationCorrectionPositionThreshold_ > 0) {
      localizationResultIsReasonable &= (positionCorrectionNorm < localizationCorrectionPositionThreshold_);
    }
    if (localizationCorrectionRotationThreshold_ > 0) {
      localizationResultIsReasonable &= (rotationCorrectionNorm < localizationCorrectionRotationThreshold_);
    }

    if (localizationResultIsReasonable) {
      localizationOperationalStatus_ = OperationalStatusCode::OK;
    } else {
      MELO_WARN_STREAM("Localization correction too large.");
      localizationOperationalStatus_ = OperationalStatusCode::WARN;
    }
    consecutiveLocalizationFails_ = 0;
  } else {
    MELO_WARN_STREAM("Localization could not converge.");
    localizationOperationalStatus_ = OperationalStatusCode::ERROR;

    // Increase consecutive localization fails counter.
    consecutiveLocalizationFails_++;
    MELO_DEBUG_STREAM("Consecutive localization fails: " << consecutiveLocalizationFails_);
  }

  // Reset if there have been too many consecutive localization fails. Don't reset if parameter is 0.
  if ((consecutiveLocalizationFails_ >= consecutiveLocalizationFailsBeforeReset_) && (consecutiveLocalizationFailsBeforeReset_ > 0)) {
    // Reset everything and start over
    consecutiveLocalizationFails_ = 0;
    this->reset();
  }
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::updateRobotPoseInTime() {
  MELO_DEBUG_STREAM("Updating robot pose in time.");

  // Update the pose in time using estimator information from tf.
  // t0: Time of the last ICP result.
  // t1: Time of the newest odometry information.
  try {
    // Get t0.
    ros::Time t0;
    {
      std::lock_guard<std::mutex> lock(this->robotPoseInMapMutex_);
      t0 = this->robotPoseInMap_.header.stamp;
    }

    // Get the robot to map transformation at t0.
    HomTransformQuat T_MRt0;
    kindr_ros::convertFromRosGeometryMsg(this->robotPoseInMap_.pose.pose, T_MRt0);

    // Get the transformation of the robot frame from t1 to t0.
    // Note: Use the inverse to get the stamp to be t1.
    geometry_msgs::TransformStamped tfMsg;
    tfMsg = this->tfBuffer_.lookupTransform(this->coordinateFrames_.robotFrameId(), ros::Time(0), this->coordinateFrames_.robotFrameId(),
                                            t0, this->coordinateFrames_.odometryFrameId(), ros::Duration(waitForTfDuration_));
    tf::StampedTransform robotMotionTf;
    tf::transformStampedMsgToTF(tfMsg, robotMotionTf);
    HomTransformQuat T_Rt0Rt1;
    kindr_ros::convertFromRosTf(robotMotionTf.inverse(), T_Rt0Rt1);

    // Get t1.
    const ros::Time t1 = robotMotionTf.stamp_;

    // Calculate the robot to map transformation at t1.
    HomTransformQuat T_MRt1 = T_MRt0 * T_Rt0Rt1;
    T_MRt1.getRotation().fix();

    // Convert back to ROS.
    {
      std::lock_guard<std::mutex> lock(this->robotPoseInMapMutex_);
      this->initialGuessRobotPoseInMap_.header.stamp = t1;
      kindr_ros::convertToRosGeometryMsg(T_MRt1, this->initialGuessRobotPoseInMap_.pose);
    }

    // Output data for debugging.
    MELO_DEBUG_STREAM("Robot pose at " << t0 << " s:" << std::endl << T_MRt0);
    MELO_DEBUG_STREAM("Robot motion over " << t1 - t0 << " s:" << std::endl << T_Rt0Rt1);
    MELO_DEBUG_STREAM("Updated robot pose at " << t1 << " s:" << std::endl << T_MRt1);
  } catch (const tf::TransformException& exception) {
    MELO_ERROR_STREAM("Caught a tf exception while updating the resulting pose: " << exception.what());
  }
}

template <typename MapBuilderType>
bool SlamNodeT<MapBuilderType>::updateCorrectionTransformFromRobotPoseInMap() {
  geometry_msgs::PoseWithCovarianceStamped robotPoseInMap;
  {
    std::lock_guard<std::mutex> lock(this->robotPoseInMapMutex_);
    robotPoseInMap = this->robotPoseInMap_;
  }

  // Update the transforms.
  try {
    // Get the robot to map transformation.
    HomTransformQuat T_MR;
    kindr_ros::convertFromRosGeometryMsg(robotPoseInMap.pose.pose, T_MR);

    // Get the odometry to robot transformation.
    geometry_msgs::TransformStamped odometryToRobotTfMsg =
        this->tfBuffer_.lookupTransform(this->coordinateFrames_.robotFrameId(), this->coordinateFrames_.odometryFrameId(),
                                        robotPoseInMap.header.stamp, ros::Duration(waitForTfDuration_));
    tf::StampedTransform odometryToRobotTf;
    tf::transformStampedMsgToTF(odometryToRobotTfMsg, odometryToRobotTf);
    HomTransformQuat T_RO;
    kindr_ros::convertFromRosTf(odometryToRobotTf, T_RO);

    // Compute the odometry to map transformation.
    HomTransformQuat T_MO = T_MR * T_RO;
    T_MO.getRotation().fix();

    // Convert the transformation back to TF.
    tf::Transform transformOdometryToMap;
    kindr_ros::convertToRosTf(T_MO, transformOdometryToMap);
    tf::StampedTransform mapToOdometryStampedTransform_ =
        tf::StampedTransform(transformOdometryToMap.inverse(), robotPoseInMap.header.stamp, this->coordinateFrames_.odometryFrameId(),
                             this->coordinateFrames_.mapFrameId());

    // Convert to ROS message.
    const geometry_msgs::PoseStamped poseStamped = PointMatcher_ros::tfToPose(mapToOdometryStampedTransform_);

    // Update map->odom pose.
    {
      std::lock_guard<std::mutex> lock(this->robotPoseInMapMutex_);
      this->mapPoseInOdometry_.header = poseStamped.header;
      this->mapPoseInOdometry_.pose.pose = poseStamped.pose;
    }

    return true;
  } catch (const tf::TransformException& exception) {
    MELO_ERROR_STREAM("Caught a tf exception while updating the "
                      << "map to odometry transformation: " << exception.what());
    return false;
  }
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::loadMapCache() {
  // Prepare the map cache for localization.
  const PmTf tfRobotToMap = PointMatcher_ros::poseToPmTf(this->initialGuessRobotPoseInMap_, this->coordinateFrames_.robotFrameId());
  this->mapBuilder_.consolidateMapForLocalization(tfRobotToMap);

  activeMapCache_ = this->mapBuilder_.getMapInCache();
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::clearCachedData() {
  // Clear localization map cache.
  activeMapCache_.clear();
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::publishGeneralSlamTopics() const {
  // Publish the initial guess of the robot pose in map.
  this->publishInitialGuess();

  // Publish the visualizable robot pose.
  this->publishVisualizableRobotPose();

  if (localizationIsSuccessful_) {
    // If enabled, publish the resulting localization.
    if (this->publishPoseIsEnabled_) {
      // Publish the robot pose in the map.
      this->publishRobotPoseInMap();

      // Publish the pose of the map frame in the odom frame.
      this->publishMapPose();
    }
  }

  // Publish the matched input point cloud.
  publishMatchedInputPointCloud();

  // Publish map.
  publishMap(false);
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::publishMap(const bool /*publishHighBandwidthTopics*/) const {
  if (mapStaticPublisher_.getNumSubscribers() == 0u && !mapStaticPublisher_.isLatched() && mapDynamicPublisher_.getNumSubscribers() == 0u &&
      !mapDynamicPublisher_.isLatched()) {
    return;
  }

  const ros::Time timestamp = ros::Time::now();

  if (mapStaticPublisher_.getNumSubscribers() > 0u || mapStaticPublisher_.isLatched()) {
    MELO_DEBUG_STREAM("Publishing the static map.");
    mapStaticPublisher_.publish(activeMapCache_.staticMapPointCloudInMapFrame_.toRosMsg(timestamp));
  }

  if (mapDynamicPublisher_.getNumSubscribers() > 0u || mapDynamicPublisher_.isLatched()) {
    MELO_DEBUG_STREAM("Publishing the dynamic map.");
    mapDynamicPublisher_.publish(activeMapCache_.dynamicMapPointCloudInMapFrame_.toRosMsg(timestamp));
  }
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::publishMatchedInputPointCloud() const {
  // Transform the input point cloud to the map frame and publish it.
  if (matchedInputPointCloudPublisher_.getNumSubscribers() > 0u || matchedInputPointCloudPublisher_.isLatched()) {
    MELO_DEBUG_STREAM("Publishing the matched input point cloud.");
    matchedInputPointCloudPublisher_.publish(matchedInputPointCloud_.toRosMsg());
  }
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::publishNodeState() const {
  if (this->nodeStatePublisher_.getNumSubscribers() > 0u || this->nodeStatePublisher_.isLatched()) {
    auto nodeState = boost::make_shared<slam_common_msgs::NodeState>();
    nodeState->header.stamp = ros::Time::now();

    // Node.
    nodeState->node_uptime.data = ros::Time::now() - this->timeOfStartingNode_;

    // Operation modes.
    nodeState->initial_guess_is_received = static_cast<uint8_t>(this->initialGuessIsReceived_);
    nodeState->input_point_cloud_matching_is_enabled = static_cast<uint8_t>(this->localizationIsEnabled_);
    nodeState->localization_is_enabled = static_cast<uint8_t>(this->publishPoseIsEnabled_);
    nodeState->mapping_is_enabled = static_cast<uint8_t>(this->mappingIsEnabled_);
    nodeState->loop_detection_is_enabled = static_cast<uint8_t>(this->mapBuilder_.isLoopDetectionEnabled());
    nodeState->reset_is_ongoing = static_cast<uint8_t>(this->resetIsOngoing_);

    // Input metrics.
    nodeState->input_point_cloud_age = inputPointCloudAge_;
    nodeState->input_point_cloud_size = inputPointCloudSize_;

    // Localization metrics.
    nodeState->localization_status = static_cast<uint8_t>(localizationOperationalStatus_);
    nodeState->registration_overlap = icpOverlap_;
    nodeState->localization_is_successful = static_cast<uint8_t>(this->localizationIsSuccessful_ && this->publishPoseIsEnabled_);

    // Mapping metrics.
    nodeState->mapping_status = static_cast<uint8_t>(this->mapBuilder_.operationalStatus());
    nodeState->map_point_cloud_size = this->mapBuilder_.getMapSize();
    nodeState->static_map_point_cloud_size = this->mapBuilder_.getStaticMapSize();
    nodeState->mapping_queue_size = this->mapBuilder_.getMappingQueueSize();
    nodeState->number_of_maps = this->mapBuilder_.getNumberOfMaps();

    // Timing.
    nodeState->time_between_input_point_clouds = timeBetweenInputPointClouds_;
    nodeState->time_used_for_localization = this->timeUsedForLocalization_;
    nodeState->time_used_for_mapping = this->mapBuilder_.timeUsedForMapping();
    // TODO(ynava) Check if this is the best way of calculating idle time. SLAM is multi-threaded and asynchronous.
    nodeState->time_idle =
        nodeState->time_between_input_point_clouds - (nodeState->time_used_for_localization + nodeState->time_used_for_mapping);

    MELO_DEBUG_STREAM("Publishing the node state.");
    this->nodeStatePublisher_.publish(nodeState);
  }
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::setUpGeneralPublishers() {
  // Node state.
  slam_common_ros::advertise<slam_common_msgs::NodeState>(this->nodeHandle_, this->nodeStatePublisher_, "node_state");

  // Point clouds.
  slam_common_ros::advertise<sensor_msgs::PointCloud2>(this->nodeHandle_, matchedInputPointCloudPublisher_, "input_point_cloud_matched");
  slam_common_ros::advertise<sensor_msgs::PointCloud2>(this->nodeHandle_, mapStaticPublisher_, "map_static");
  slam_common_ros::advertise<sensor_msgs::PointCloud2>(this->nodeHandle_, mapDynamicPublisher_, "map_dynamic");
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::setUpGeneralSubscribers() {
  // Subscribers.
}

template <typename MapBuilderType>
void SlamNodeT<MapBuilderType>::setUpGeneralServices() {
  // Node configuration.
  slam_common_ros::advertiseService(this->nodeHandle_, readConfigFilesServer_, "read_config_files",
                                    &SlamNodeT<MapBuilderType>::readConfigFilesCb, this);
}

}  // namespace icp_tools_common
