#pragma once

// std
#include <atomic>
#include <memory>

// pointmatcher
#include <pointmatcher/PointMatcher.h>

// pointmatcher_ros
#include <pointmatcher_ros/PmTf.h>
#include <pointmatcher_ros/StampedPointCloud.h>

// ros
#include <ros/ros.h>

// std srvs
#include <std_srvs/Empty.h>

// sensor msgs
#include <sensor_msgs/PointCloud2.h>

// slam common ros
#include <slam_common_ros/SlamInterface.hpp>

// icp tools common
#include "icp_tools_common/usings.hpp"

// slam common ros
#include <slam_common_ros/SlamInterface.hpp>

namespace icp_tools_common {

/*!
 * SLAM Node (template).
 *
 *  Functionality
 *  - Build a point cloud map.
 *  - Localize a robot within the map.
 *
 *  Threading model:
 *    There are two threads for localization and mapping:
 *      1) A ROS thread that handles all ROS callbacks as well as the localization step.
 *      2) An std thread that runs the mapping routine and is notified by the ROS thread if mapping needs to be done.
 *    This parallel execution avoids slowing down the localization, specially if mapping is enabled.
 *    Furthermore, there is an additional thread in charge of publishing the slam system node state.
 */
template <typename MapBuilderType>
class SlamNodeT : public SlamInterface<MapBuilderType> {
 public:
  /*!
   * @copydoc icp_tools_common::SlamInterface::SlamInterface(ros::NodeHandle& nodeHandle)
   */
  explicit SlamNodeT(ros::NodeHandle& nodeHandle);

  /*!
   * @brief Input point cloud callback function.
   *
   * @param inputPointCloudRos  Input point cloud as a ROS msg type.
   */
  virtual void inputPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& inputPointCloudRos);

 protected:
  /*!
   * @copydoc slam_common_ros::SlamInterface::readGeneralSlamParameters()
   */
  bool readGeneralSlamParameters() override;

  /*!
   * @copydoc slam_common_ros::SlamInterface::readLocalizationParameters()
   */
  bool readLocalizationParameters() override;

  /*!
   * @copydoc slam_common_ros::SlamInterface::readLocalizationParameters()
   */
  bool readMappingParameters() override;

  /*!
   * @brief Read the config files.
   *
   * @return True   If successful.
   */
  virtual bool readConfigFiles();

  /*!
   * Read the config files.
   * @param request   request message.
   * @param response  response message.
   * @return  True    If successful.
   */
  bool readConfigFilesCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * @copydoc slam_common_ros::SlamInterface::setUpLocalizationSubscribers()
   */
  void setUpLocalizationSubscribers() override;

  /*!
   * @copydoc slam_common_ros::SlamInterface::shutdownLocalizationSubscribers()
   */
  void shutdownLocalizationSubscribers() override;

  /*!
   * @copydoc slam_common_ros::SlamInterface::resetLocalizationFlags()
   */
  void resetLocalizationFlags() override;

  /*!
   * @brief Process the input point cloud, by performing localization, mapping and running other routines.
   *
   * @param inputPointCloud   The input point cloud.
   */
  virtual bool processInputPointCloud(const PmStampedPointCloud& inputPointCloud) = 0;

  /*!
   * @brief Get the initial guess for the transformation from robot to map.
   *
   * @param desiredTime   Desired time.
   * @param tfRobotToMap  Will contain the initial guess, if a valid initial guess is available.
   * @return True         If a valid initial guess is available.
   */
  virtual bool getRobotToMapInitialGuess(const ros::Time& desiredTime, PmTf& tfRobotToMap);

  /*!
   * @brief Localize the robot within the map.
   *
   * @param inputPointCloudRos        ROS input point cloud.
   * @param tfRobotToMapInitialGuess  The initial guess of the robot pose in the map.
   * @param covariance                Covariance of the localization estimate, will be modified.
   * @return True                     If successful.
   */
  virtual bool localize(const PmStampedPointCloud& inputPointCloud, const PmTf& tfRobotToMapInitialGuess, PmTf& tfRobotToMap,
                        PmMatrix& covariance);

  /**
   * @brief Process the map point cloud to fulfill localization requirements.
   *
   * @param mapPointCloud             Map point cloud in the map frame.
   * @param tfRobotToMapInitialGuess  Initial guess on the robot pose in the map frame.
   * @return PmStampedPointCloud      Processed point cloud in the map frame.
   */
  virtual PmStampedPointCloud processMapForLocalization(const PmStampedPointCloud& mapPointCloud, const PmTf& tfRobotToMapInitialGuess);

  /*!
   * @brief Matches the input point cloud to the map.
   *
   * @param inputPointCloud           Input point cloud.
   * @param mapPointCloud             Point cloud of reference map, in the map frame.
   * @param tfRobotToMapInitialGuess  Initial guess for transformation from robot to map.
   * @param tfRobotToMap              If ICP matching succeeds, it will contain the resulting transformation between map and robot frames.
   * @param covariance                If ICP matching succeeds, it will contain the covariance.
   * @return                          True if successful.
   */
  virtual bool match(const PmStampedPointCloud& inputPointCloud, const PmStampedPointCloud& mapPointCloud,
                     const PmTf& tfRobotToMapInitialGuess, PmTf& tfRobotToMap, PmMatrix& covariance);

  /*!
   * @brief Align the pose in map frame with the gravity vector.
   *
   * @param tfRobotToMap  Transformation from robot to map, will be modified.
   * @return True         If successful.
   */
  virtual bool alignPoseInMapWithGravity(PmTf& tfRobotToMap) const;

  /*!
   * @brief Validates the localization result and sets flags which indicate the localization status.
   *
   * @param localizationIsSuccessful  Whether localization was successful.
   * @param tfRobotToMap              Robot pose in map according to localization.
   * @param tfRobotToMapInitialGuess  Initial guess on the robot pose in map.
   */
  virtual void validateLocalizationResult(const bool localizationIsSuccessful, const PmTf& tfRobotToMap,
                                          const PmTf& tfRobotToMapInitialGuess, const PmMatrix& covariance);

  /*!
   * @brief Take the resulting robot pose in map and update it using odometry data from the state estimator.
   *
   */
  virtual void updateRobotPoseInTime();

  /*!
   * @brief Update the map->odom correction transform from estimated map->robot transform.
   *
   * @return True if successful.
   */
  virtual bool updateCorrectionTransformFromRobotPoseInMap();

  /*!
   * @copydoc slam_common_ros::SlamInterface::loadMapCache()
   */
  void loadMapCache() override;

  /*!
   * @copydoc slam_common_ros::SlamInterface::clearCachedData()
   */
  void clearCachedData() override;

  /*!
   * @copydoc slam_common_ros::SlamInterface::publishGeneralSlamTopics()
   */
  void publishGeneralSlamTopics() const override;

  /*!
   * @copydoc slam_common_ros::SlamInterface::publishMap(const bool publishHighBandwidthTopics)
   */
  void publishMap(const bool publishHighBandwidthTopics) const override;

  /*!
   * Publish the matched input cloud in map.
   */
  void publishMatchedInputPointCloud() const;

  /*!
   * @copydoc slam_common_ros::SlamInterface::publishNodeState()
   */
  void publishNodeState() const override;

  //! Input point cloud subscriber.
  ros::Subscriber inputPointCloudSubscriber_;
  //! Matched input point cloud publisher.
  ros::Publisher matchedInputPointCloudPublisher_;
  //! Map (static points) publisher.
  ros::Publisher mapStaticPublisher_;
  //! Map (dynamic points) publisher.
  ros::Publisher mapDynamicPublisher_;

  //! Read config files server.
  ros::ServiceServer readConfigFilesServer_;

  //! Folder containing the configuration files.
  std::string configFilesFolder_;
  //! Name of the file containing the ICP configuration.
  std::string icpConfigFileName_;
  //! Name of the file containing the map filters.
  std::string mapFiltersFileName_;

  //! ICP pipeline.
  PmIcp icp_;

  //! Maximal range of the sensor, used for localization and mapping [m].
  float maxSensorRange_ = 60.0;
  //! Before running the localization step, the map is cut around the initial guess.
  //! The radius is determined by the max sensor range multiplied by this factor [-].
  float maxSensorRangeScalingToCutMap_ = 1.2;

  //! Use tfs as initial guess.
  bool useTf_ = false;
  //! Duration to wait after enabling publish pose to use tfs as initial guess [s].
  double updateTfDuration_ = 2.0;
  //! Duration to wait for the tf [s].
  double waitForTfDuration_ = 0.1;
  //! Correct the ICP result such that the map frame is aligned with gravity.
  bool enforceMapGravityAlignment_ = true;
  //! Update the resulting pose with the relative motion of the robot during the localization procedure.
  bool updateRobotPoseInTime_ = false;

  //! Minimum map size to run ICP (after applying the filters).
  unsigned int minMapSizeForLocalization_ = 0;
  //! Minimum input point cloud size to run ICP.
  unsigned int minInputPointCloudSizeForLocalization_ = 0;

  //! ICP overlap [0,1].
  std::atomic<double> icpOverlap_;
  //! Minimal overlap to use ICP results [0,1].
  double minOverlapForLocalization_ = 0.0;

  //! Operational status of the localization process.
  OperationalStatusCode localizationOperationalStatus_;
  //! Localization is successful.
  std::atomic<bool> localizationIsSuccessful_;

  //! Cache for localization map.
  PointCloudMapCache activeMapCache_;

  //! The initial guess of the robot pose in the map frame.
  PmTf tfRobotToMapInitialGuess_;
  //! Matched input point cloud in the map frame.
  PmStampedPointCloud matchedInputPointCloud_;

  //! Age of the input point cloud on reception.
  std::atomic<double> inputPointCloudAge_;
  //! Size of the input point cloud.
  std::atomic<unsigned int> inputPointCloudSize_;
  //! Stamp of current input point cloud message.
  ros::Time currentInputPointCloudStamp_;
  //! Stamp of last input point cloud message.
  ros::Time lastInputPointCloudStamp_;
  //! Time between input point cloud.
  std::atomic<double> timeBetweenInputPointClouds_;
  //! Time used for localization.
  std::atomic<double> timeUsedForLocalization_;

  //! Number of consecutive localization fails before resetting the node (0 means never reset)
  unsigned int consecutiveLocalizationFailsBeforeReset_ = 0;
  //! Count of current consecutive localization failures
  std::atomic<unsigned int> consecutiveLocalizationFails_;
  //! Threshold used to determine if a localization correction is reasonable, based on its change in position.
  double localizationCorrectionPositionThreshold_ = 0.0;
  //! Threshold used to determine if a localization correction is reasonable, based on its change in rotation.
  double localizationCorrectionRotationThreshold_ = 0.0;

 private:
  /*!
   * @brief Set up ROS Publishers.
   *
   */
  void setUpGeneralPublishers();

  /*!
   * @brief Set up ROS Subscribers.
   *
   */
  void setUpGeneralSubscribers();

  /*!
   * @brief Set up ROS Services
   *
   */
  void setUpGeneralServices();
};

}  // namespace icp_tools_common

#include "icp_tools_common/SlamNodeT.tpp"
