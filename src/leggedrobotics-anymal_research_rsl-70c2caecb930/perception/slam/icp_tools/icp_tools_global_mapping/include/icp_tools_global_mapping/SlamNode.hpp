#pragma once

// icp tools common
#include <icp_tools_common/SlamNodeT.hpp>

// icp tools global mapping
#include "icp_tools_global_mapping/PointCloudGlobalMapBuilder.hpp"
#include "icp_tools_global_mapping/usings.hpp"

namespace icp_tools_global_mapping {

class SlamNode : public icp_tools_common::SlamNodeT<PointCloudGlobalMapBuilder> {
 public:
  /*!
   * @copydoc icp_tools_common::SlamNodeT::SlamNode(ros::NodeHandle& nodeHandle)
   */
  explicit SlamNode(ros::NodeHandle& nodeHandle);

  /*!
   * @brief Initializes the SLAM system.
   *
   * @return True if successful.
   */
  void initSlamNode();

  /*!
   * @copydoc icp_tools_common::SlamNodeT::processInputPointCloud(const PmStampedPointCloud& inputPointCloud)
   */
  bool processInputPointCloud(const PmStampedPointCloud& inputPointCloud) override;
};

}  // namespace icp_tools_global_mapping
