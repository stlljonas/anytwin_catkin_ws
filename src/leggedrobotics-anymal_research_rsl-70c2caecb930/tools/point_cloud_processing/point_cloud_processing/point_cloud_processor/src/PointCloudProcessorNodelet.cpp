
#include "point_cloud_processor/PointCloudProcessorNodelet.hpp"

// ROS pluginlib
#include <pluginlib/class_list_macros.h>

namespace point_cloud_processor {

/*!
 * Default Constructor.
 */
PointCloudProcessor::PointCloudProcessor() : tfListener_(tfBuffer_), externalSubscribersCheckPeriod_(1.0) {}

void PointCloudProcessorNodelet::onInit() {
  nodeHandle_ = getMTPrivateNodeHandle();
  NODELET_DEBUG("Initializing nodelet...");
  initialize();
}

}  // namespace point_cloud_processor

PLUGINLIB_EXPORT_CLASS(point_cloud_processor::PointCloudProcessorNodelet, nodelet::Nodelet)