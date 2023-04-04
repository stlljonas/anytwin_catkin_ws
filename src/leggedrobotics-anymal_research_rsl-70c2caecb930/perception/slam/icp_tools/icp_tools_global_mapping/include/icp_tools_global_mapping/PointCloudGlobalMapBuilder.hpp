#pragma once

// C++ standard library
#include <memory>
#include <mutex>

// slam point cloud mapping
#include <slam_point_cloud_mapping/PointCloudMapBuilderBase.hpp>
#include <slam_point_cloud_mapping/macros.hpp>

// icp tools global mapping
#include "icp_tools_global_mapping/usings.hpp"

namespace icp_tools_global_mapping {

struct PointCloudGlobalMapBuilderParameters {
  //! Maximal distance between two points to consider them overlapping [m]. Determines map density.
  float maxDistanceOverlappingPoints_ = 0.04;

  //! Maximal overlap to add the input point cloud to the map [0,1].
  double maxOverlapForMapping_ = 1.0;

  //! Maximal range of the sensor, used for mapping [m].
  float maxSensorRange_ = 60.0;

  //! Remove dynamic points from the map.
  bool removeDynamicPointsAfterExtendingMap_ = false;
};

/**
 * Global Map Builder
 *  This class represents an encapsulation of the mapping component of ANYmal 2' Localization and Mapping (LAM).
 *
 *  Functionality:
 *  - Global mapping. It can build a static, global map, employing the output from a localization module (pose and input point cloud in the
 *    robot frame)
 *  - Trigger dynamic point estimation whenever the map is extended.
 *  - Retrieve the map static/dynamic points when queried by another module.
 *  - Load maps saved as PLY files.
 *  - Save maps to PLY files.
 *  - Clear the global map and restart the mapping process.
 *
 *  Threading model:
 *    The Global Map Builder extends the current map in an asynchronous routine, which is designed to be run in a separate thread,
 *    waiting for input from localization. The ownership of the thread is expected to be handled by an external SLAM module, that
 *    coordinates localization and mapping.
 */
class PointCloudGlobalMapBuilder : public PointCloudMapBuilderBase {
 public:
  using Ptr = std::unique_ptr<PointCloudGlobalMapBuilder>;
  using Parameters = PointCloudGlobalMapBuilderParameters;

  /**
   * @brief Default Constructor. Initializes the global map builder with default parameters.
   *
   */
  PointCloudGlobalMapBuilder();

  /**
   * @brief Construct a new Global Map Builder object.
   *
   * @param parameters The parameters for building the global map.
   */
  explicit PointCloudGlobalMapBuilder(const Parameters parameters);

  /**
   * @brief Get const reference to global map builder parameters.
   *
   * @return The parameters for building the map.
   */
  const Parameters& parameters() const { return parameters_; }

  /**
   * @brief Set global map builder parameters.
   *
   * @param parameters The new parameters to set.
   */
  void setParameters(const Parameters& parameters) { parameters_ = parameters; };

  /**
   * @copydoc slam_common::MapBuilderBase::getNumberOfMaps()
   */
  size_t getNumberOfMaps() const override;

  /**
   * @copydoc slam_common::MapBuilderBase::isLoopDetectionEnabled()
   */
  bool isLoopDetectionEnabled() const override;

  /**
   * @copydoc icp_tools_common::PointCloudMapBuilderBase::consolidateMapForLocalization(const PmTf& tfRobotToMap)
   */
  void consolidateMapForLocalization(const PmTf& tfRobotToMap) override;

  /**
   * @copydoc slam_common::MapBuilderBase::processDataInMappingQueue()
   */
  void processDataInMappingQueue() override;

  /**
   * Extend the map with a point cloud. This method is called within the mapping thread.
   *
   * @param pointCloudInRobotFrame Point cloud to add to the map.
   * @param tfRobotToMap           Transformation from robot to map.
   * @param map                    Map to extend.
   * @return True                  if successful.
   */
  bool extendMap(const PmStampedPointCloud& pointCloudInRobotFrame, const PmTf& tfRobotToMap, PmStampedPointCloud& map);

  /**
   * @copydoc slam_common::MapBuilderBase::loadMapFromFilesystem(const std::string& filePath, const ros::Time& stamp)
   */
  bool loadMapFromFilesystem(const std::string& filePath, const ros::Time& stamp) override;

  /**
   * @copydoc slam_common::MapBuilderBase::saveMapToFilesystem(const std::string& absolutePath)
   */
  bool saveMapToFilesystem(const std::string& absolutePath) const override;

  /**
   * @copydoc slam_common::MapBuilderBase::clearMap(const ros::Time& stamp)
   */
  void clearMap(const ros::Time& stamp) override;

  /**
   * @copydoc icp_tools_common::PointCloudMapBuilderBase::setUpFromRos(ros::NodeHandle& nodeHandle)
   */
  bool setUpFromRos(ros::NodeHandle& nodeHandle) override;

 protected:
  Parameters parameters_;

  //! Mutex to lock the map_ object.
  mutable std::mutex mapMutex_;
  //! Map point cloud.
  PmStampedPointCloud map_;
};

}  // namespace icp_tools_global_mapping