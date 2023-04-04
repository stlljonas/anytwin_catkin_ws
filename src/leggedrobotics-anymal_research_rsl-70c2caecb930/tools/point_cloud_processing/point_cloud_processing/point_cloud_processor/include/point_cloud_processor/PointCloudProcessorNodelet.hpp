/*
 * PointCloudProcessorNodelet.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

#include "point_cloud_processor/PointCloudProcessor.hpp"

// Nodelets
#include <nodelet/nodelet.h>

namespace point_cloud_processor {

/**
 * @brief PointCloudProcessorNodelet
 *  An instantiation of a PointCloudProcessor object inside a ROS nodelet.
 */
class PointCloudProcessorNodelet : public nodelet::Nodelet, public PointCloudProcessor {
 public:
  /*!
   * Default Constructor.
   */
  PointCloudProcessorNodelet() = default;

  /*!
   * Method for initializing nodelet.
   */
  void onInit() override;
};

}  // namespace point_cloud_processor
