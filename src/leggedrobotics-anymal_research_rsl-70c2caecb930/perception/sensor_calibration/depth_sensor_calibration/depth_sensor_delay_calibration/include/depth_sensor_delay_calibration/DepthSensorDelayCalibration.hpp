#pragma once


// std
#include <memory>
#include <vector>

// ros
#include <ros/ros.h>

// tf
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// tf2 ros
#include <tf2_ros/transform_listener.h>

// sensor msgs
#include <sensor_msgs/PointCloud2.h>

// any msgs
#include <any_msgs/Toggle.h>

// pointmatcher ros
#include <pointmatcher_ros/StampedPointCloud.h>

// depth sensor delay calibration
#include "depth_sensor_delay_calibration/OptimizationNelderMead.hpp"


namespace depth_sensor_delay_calibration {


class DepthSensorDelayCalibration
{
protected:
  using PmPointCloud = PointMatcher_ros::StampedPointCloud;
  using PmTf = PointMatcher_ros::PmTf;
  using PmIcp = PointMatcher_ros::PmIcp;
  using PmPointCloudFilters = PointMatcher_ros::PmPointCloudFilters;
  using Optimizer = OptimizationNelderMead<1>;
  using Parameter = Optimizer::Vector;

  // Error in case the error calculation fails. Infinity could mess up the optimization.
  const double bigError_ = 1.0;

  ros::NodeHandle nodeHandle_;

  ros::Subscriber pointCloudSubscriber_;
  ros::ServiceServer toggleRecordingServer_;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  std::string fixedFrameId_;
  PmPointCloudFilters pointCloudFilters_;
  unsigned int minPointsForIcp_ = 0;
  PmIcp icp_;

  bool isRecording_ = false;
  std::vector<sensor_msgs::PointCloud2ConstPtr> pointCloudsRos_;
  std::vector<PmPointCloud> pointClouds_;

public:
  DepthSensorDelayCalibration(ros::NodeHandle& nodeHandle);

protected:
  bool readPointCloudFiltersConfigFile(const std::string& filePath);
  bool readIcpConfigFile(const std::string& filePath);

  void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pointCloud);

  bool toggleRecording(any_msgs::ToggleRequest& req, any_msgs::ToggleResponse& res);

  bool computeOffset(double& offset);
  double computeErrorFromParameter(const Parameter parameter);
  double computeError(const double offset);
  double computePointCloudMatchingError(const PmPointCloud& pointCloud1, const PmPointCloud& pointCloud2);
  double computeErrorFromTf(const PmTf& tf);

  void computeTimeStampStatistics(double& dtMean, double& dtStdDev, double& dtMin, double& dtMax) const;
};


} // depth_sensor_delay_calibration
