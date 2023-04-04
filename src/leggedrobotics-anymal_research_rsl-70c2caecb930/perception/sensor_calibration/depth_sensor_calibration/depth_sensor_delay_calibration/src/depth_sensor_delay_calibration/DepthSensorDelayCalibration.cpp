// param io
#include <param_io/get_param.hpp>

// pointmatcher ros
#include <pointmatcher_ros/helper_functions.h>

// depth sensor delay calibration
#include "depth_sensor_delay_calibration/DepthSensorDelayCalibration.hpp"


namespace depth_sensor_delay_calibration {


DepthSensorDelayCalibration::DepthSensorDelayCalibration(ros::NodeHandle& nodeHandle)
: nodeHandle_(nodeHandle)
{
  bool success = true;

  double tfBufferLength = 0.0;
  std::string icpFile;
  std::string pointCloudFiltersFile;

  success &= param_io::getParam(nodeHandle_, "fixed_frame_id", fixedFrameId_);
  success &= param_io::getParam(nodeHandle_, "tf_buffer_length", tfBufferLength);
  success &= param_io::getParam(nodeHandle_, "point_cloud_filters_file", pointCloudFiltersFile);
  success &= param_io::getParam(nodeHandle_, "min_points_for_icp", minPointsForIcp_);
  success &= param_io::getParam(nodeHandle_, "icp_file", icpFile);

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(tfBufferLength));
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  success &= readPointCloudFiltersConfigFile(pointCloudFiltersFile);
  success &= readIcpConfigFile(icpFile);

  if (!success) {
    ros::requestShutdown();
  }

  pointCloudSubscriber_ = nodeHandle_.subscribe(
      param_io::param<std::string>(nodeHandle_, "point_cloud_topic", "point_cloud"),
      1, &DepthSensorDelayCalibration::pointCloudCb, this);

  toggleRecordingServer_ = nodeHandle_.advertiseService(
      "toggle_recording", &DepthSensorDelayCalibration::toggleRecording, this);

  ROS_INFO("Depth sensor delay calibration node started.");
}

bool DepthSensorDelayCalibration::readPointCloudFiltersConfigFile(const std::string& filePath)
{
  std::ifstream ifFilePath(filePath.c_str());
  if (!ifFilePath.good()) {
    ROS_WARN_STREAM("Cannot load point cloud filters configuration from '" << filePath << "'.");
    return false;
  }
  pointCloudFilters_ = PmPointCloudFilters(ifFilePath);
  return true;
}

bool DepthSensorDelayCalibration::readIcpConfigFile(const std::string& filePath)
{
  std::ifstream ifFilePath(filePath.c_str());
  if (!ifFilePath.good()) {
    ROS_WARN_STREAM("Cannot load ICP configuration from '" << filePath << "'.");
    return false;
  }
  icp_.loadFromYaml(ifFilePath);
  return true;
}

void DepthSensorDelayCalibration::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pointCloud)
{
  // Do not save the point cloud if not recording.
  if (!isRecording_) {
    return;
  }

  // Save the point cloud.
  pointCloudsRos_.push_back(pointCloud);
}

bool DepthSensorDelayCalibration::toggleRecording(any_msgs::ToggleRequest& req, any_msgs::ToggleResponse& res)
{
  res.success = true;
  if (req.enable) {
    ROS_INFO_STREAM("Started calibration. Collecting point clouds ...");
    isRecording_ = true;
    pointCloudsRos_.clear();
    return true;
  }
  else {
    ROS_INFO_STREAM("Stopped calibration. Optimizing time stamp offset ...");
    ROS_INFO_STREAM("Received " << pointCloudsRos_.size() << " point clouds.");

    // Pre-process point clouds for optimization.
    ROS_INFO_STREAM("Filtering point clouds ...");
    pointClouds_.clear();
    for (const sensor_msgs::PointCloud2ConstPtr& pointCloudRos : pointCloudsRos_) {
      // Conversion.
      PmPointCloud pointCloud = PmPointCloud::FromRosMsg(*pointCloudRos);

      // Filtering.
      const unsigned int sizeBeforeFiltering = pointCloud.getSize();
      pointCloud.filter(pointCloudFilters_);
      const unsigned int sizeAfterFiltering = pointCloud.getSize();
      ROS_INFO_STREAM("Size before and after filtering: " << sizeBeforeFiltering << " -> " << sizeAfterFiltering);

      // Check size.
      if (sizeAfterFiltering < minPointsForIcp_) {
        ROS_WARN_STREAM("Filtered point cloud is too small for ICP (min. " << minPointsForIcp_ << " points required).");
        continue;
      }

      // Add to point clouds for optimization.
      pointClouds_.push_back(pointCloud);
    }

    // Check if enough point clouds are available.
    const size_t minNumPointClouds = 2;
    if (pointClouds_.size() < minNumPointClouds) {
      ROS_ERROR_STREAM("Not enough point clouds for optimization (" << pointClouds_.size() <<
          " < " << minNumPointClouds << ").");
      return true;
    }

    ROS_INFO_STREAM(pointClouds_.size() << " point clouds are used for the optimization.");

    // Compute the time stamp offset.
    double offset = 0.0;
    if (!computeOffset(offset)) {
      ROS_ERROR_STREAM("The time stamp offset could not be computed.");
      return true;
    }

    // Print statistics.
    double dtMean, dtStdDev, dtMin, dtMax;
    computeTimeStampStatistics(dtMean, dtStdDev, dtMin, dtMax);
    ROS_INFO_STREAM("Statistics of the input point cloud time stamps:");
    ROS_INFO_STREAM("Mean dt:    " << dtMean);
    ROS_INFO_STREAM("Std Dev dt: " << dtStdDev);
    ROS_INFO_STREAM("Min dt:     " << dtMin);
    ROS_INFO_STREAM("Max dt:     " << dtMax);

    // Print the time stamp offset.
    ROS_INFO_STREAM("The time stamp offset is " << offset << " seconds.");
    ROS_INFO_STREAM("Add it to the time stamp of your point clouds.");
    return true;
  }
}

bool DepthSensorDelayCalibration::computeOffset(double& offset)
{
  ROS_INFO_STREAM("Starting optimizer ...");
  Optimizer optimizer;
  double error = 0.0;
  Parameter parameter = Parameter::Zero();
  Parameter parameter0 = Parameter::Zero();

  if (!optimizer.optimize(
      error,
      parameter,
      parameter0,
      std::bind(&DepthSensorDelayCalibration::computeErrorFromParameter, this, std::placeholders::_1),
      true)) {
    ROS_ERROR_STREAM("The optimization failed.");
    return false;
  }

  offset = parameter(0);

  //    for (offset = -0.3; offset <= 0.3; offset += 0.001) {
  //      computeError(offset);
  //    }

  return true;
}

double DepthSensorDelayCalibration::computeErrorFromParameter(const Parameter parameter)
{
  return computeError(parameter(0));
}

double DepthSensorDelayCalibration::computeError(const double offset)
{
  // Transform point clouds to fixed frame adding the given offset to the stamp.
  std::vector<PmPointCloud> pointCloudsDelayedInFixedFrame;
  for (const PmPointCloud& pointCloud : pointClouds_) {
    geometry_msgs::TransformStamped tfMsg;
    try{
      tfMsg = tfBuffer_->lookupTransform(
          fixedFrameId_,
          pointCloud.header_.frame_id,
          pointCloud.header_.stamp + ros::Duration(offset));
    }
    catch (const tf2::TransformException& ex) {
      ROS_ERROR_STREAM("Caught an exception while looking up a transform: " << ex.what());
      ROS_ERROR_STREAM("Increase TF buffer length and/or make sure to fill it a bit before starting to record point clouds.");
      continue;
    }
    tf::StampedTransform tf;
    tf::transformStampedMsgToTF(tfMsg, tf);
    const PmTf tfPm = PointMatcher_ros::tfToPmTf(tf);
    PmPointCloud pointCloudDelayedInFixedFrame = pointCloud;
    pointCloudDelayedInFixedFrame.transform(tfPm);
    pointCloudsDelayedInFixedFrame.push_back(pointCloudDelayedInFixedFrame);
  }

  // Match pairs of subsequent point clouds and compute the mean matching error.
  if (pointCloudsDelayedInFixedFrame.size() < 2) {
    ROS_ERROR_STREAM("Not enough point clouds could be transformed for offset " << offset << " seconds.");
    return bigError_;
  }
  double error = 0.0;
  for (size_t i = 1; i < pointCloudsDelayedInFixedFrame.size(); i++) {
    error += computePointCloudMatchingError(
        pointCloudsDelayedInFixedFrame[i-1],
        pointCloudsDelayedInFixedFrame[i]);
  }
  error /= pointCloudsDelayedInFixedFrame.size();
  ROS_INFO_STREAM("Offset: " << offset << ", error: " << error);
  return error;
}

double DepthSensorDelayCalibration::computePointCloudMatchingError(const PmPointCloud& pointCloud1, const PmPointCloud& pointCloud2)
{
  // Run the ICP sequence.
  PmTf tf;
  try {
    tf.parameters_ = icp_.compute(pointCloud1.dataPoints_, pointCloud2.dataPoints_, PmTf().parameters_);
  }
  catch (const std::exception& exception) {
    ROS_ERROR_STREAM("Caught an exception while running ICP: " << exception.what());
    return bigError_;
  }

  // Compute the error.
  return computeErrorFromTf(tf);
}

double DepthSensorDelayCalibration::computeErrorFromTf(const PmTf& tf)
{
  //    const double translationalError = static_cast<double>(
  //        tf.parameters_.block<3,1>(0,3).squaredNorm());
  //    const double rotationalError = static_cast<double>(
  //        std::pow(Eigen::AngleAxisf(tf.parameters_.topLeftCorner<3,3>()).angle(), 2));

  // TODO This is probably too heuristic.
  const Eigen::Vector3f unitZ = Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f unitZtf = (tf.parameters_.topLeftCorner<3,3>() * unitZ).normalized();
  const double dotProduct = unitZ.dot(unitZtf);
  // Note: std::acos takes a value in [-1, 1] and returns a value in [0, pi].
  const double angle = static_cast<double>(
      std::acos(std::min(std::max(dotProduct, -1.0), 1.0)));

  return angle;
}

void DepthSensorDelayCalibration::computeTimeStampStatistics(double& dtMean, double& dtStdDev, double& dtMin, double& dtMax) const
{
  if (pointCloudsRos_.size() <= 1) {
    dtMean = std::numeric_limits<double>::quiet_NaN();
    dtStdDev = std::numeric_limits<double>::quiet_NaN();
    dtMin = std::numeric_limits<double>::quiet_NaN();
    dtMax = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  std::vector<double> dts;
  for (size_t i = 1; i < pointCloudsRos_.size(); i++) {
    dts.push_back((pointCloudsRos_[i]->header.stamp - pointCloudsRos_[i-1]->header.stamp).toSec());
  }

  dtMean = 0.0;
  dtStdDev = 0.0;
  dtMin = std::numeric_limits<double>::infinity();
  dtMax = -std::numeric_limits<double>::infinity();

  for (const double dt : dts) {
    dtMean += dt;
    dtMin = std::min(dtMin, dt);
    dtMax = std::max(dtMax, dt);
  }
  dtMean /= dts.size();

  for (const double dt : dts) {
    dtStdDev += std::pow(dt - dtMean, 2);
  }
  dtStdDev = std::sqrt(dtStdDev/dts.size());
}


} // depth_sensor_delay_calibration

