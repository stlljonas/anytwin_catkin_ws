/** \file gazebo_ros_3dlidar.h
    \brief This file defines the GazeboRos3dLidar class which simulates a 3D
           lidar in Gazebo and interfaces it to ROS.
  */

#pragma once

#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/GpuRaySensor.hh>

#include <ros/ros.h>

#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace gazebo {

  /** The GazeboRos3dLidar class interfaces a Gazebo 3D lidar with ROS.
      \brief Gazebo ROS interface to 3D lidar.
    */
  class GazeboRos3dLidar :
    public SensorPlugin {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructs object
    GazeboRos3dLidar();
    /// Destructor
    virtual ~GazeboRos3dLidar();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Implements Gazebo virtual load function
    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
    /// Overrides Gazebo init function
    virtual void Init();
    /// Overrides Gazebo reset function
    virtual void Reset();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Inits the ROS publishers
    void initPublishers();
    /// Reads parameters from the parameter server
    void readParameters();
    /// Publishes point cloud over ROS
    void publishPointCloud(const float* scans);
    /// Callback for new laser frame
    void OnNewLaserFrame(const float* scans, unsigned int width, unsigned int
      height, unsigned int depth, const std::string& format);
    /// Returns inclination angle from given index
    float getInclination(size_t index) {
#if GAZEBO_MAJOR_VERSION >= 7
      return sensor_->VerticalAngleMin().Radian() + index *
        (sensor_->VerticalAngleMax() -
        sensor_->VerticalAngleMin()).Radian() /
        (sensor_->VerticalRayCount() - 1);
#else
      return sensor_->GetVerticalAngleMin().Radian() + index *
        (sensor_->GetVerticalAngleMax() -
        sensor_->GetVerticalAngleMin()).Radian() /
        (sensor_->GetVerticalRayCount() - 1);
#endif
    }
    /// Returns azimuth angle from given index
    float getAzimuth(size_t index) {
#if GAZEBO_MAJOR_VERSION >= 7
      return sensor_->AngleMin().Radian() + index *
        (sensor_->AngleMax() -
        sensor_->AngleMin()).Radian() /
        (sensor_->RayCount() - 1);
#else
      return sensor_->GetAngleMin().Radian() + index *
        (sensor_->GetAngleMax() -
        sensor_->GetAngleMin()).Radian() /
        (sensor_->GetRayCount() - 1);
#endif
    }
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    std::shared_ptr<ros::NodeHandle> nodeHandle_;
    /// ROS base namespace for this plugin
    std::string namespace_;
    /// Diagnostic updater
    diagnostic_updater::Updater diagnosticUpdater_;
    /// ROS point cloud publisher
    ros::Publisher pointCloudPublisher_;
    /// ROS point cloud publisher topic name
    std::string pointCloudPublisherTopic_;
    /// ROS point cloud publisher frame name
    std::string pointCloudPublisherFrameId_;
    /// ROS point cloud publisher queue size
    int pointCloudPublisherQueueSize_;
    /// ROS point cloud publisher minimum frequency
    double pointCloudPublisherMinFrequency_;
    /// ROS point cloud publisher maximum frequency
    double pointCloudPublisherMaxFrequency_;
    /// ROS point cloud publisher frequency diagnostic
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>
      pointCloudPublisherFrequencyDiagnostic_;
    /// Gazebo world
    physics::WorldPtr world_;
    /// Gazebo GPU ray sensor
    sensors::GpuRaySensorPtr sensor_;
    /// Last Gazebo update time
//    common::Time lastUpdateTime_;
    /// Connection to new laser frame
    event::ConnectionPtr newLaserFrameConnection_;
    /** @}
      */

  };

}

