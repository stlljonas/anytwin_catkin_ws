#include "gazebo/GazeboRos3dLidar.hpp"

#include <iostream>
#include <cmath>
#include <fstream>

#include <boost/pointer_cast.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <gazebo/gazebo_config.h>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/Point32.h>

namespace gazebo {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  GazeboRos3dLidar::GazeboRos3dLidar() {
  }

  GazeboRos3dLidar::~GazeboRos3dLidar() {
    if (sensor_)
#if (GAZEBO_MAJOR_VERSION >= 8)
      sensor_.reset();
#else
      sensor_->DisconnectNewLaserFrame(newLaserFrameConnection_);
#endif
    if (nodeHandle_)
      nodeHandle_->shutdown();
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void GazeboRos3dLidar::Init() {
  }

  void GazeboRos3dLidar::Reset() {
  }

  void GazeboRos3dLidar::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // cast the sensor into GPU ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
    sensor_ =
      std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);
#else
    sensor_ =
        boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);
#endif
    if (!sensor_) {
        ROS_ERROR_STREAM_NAMED("gazebo_ros_3dlidar",
          "GazeboRos3dLidar requires a sensor of type gpu_ray");
      return;
    }

    // Get the world
    const auto worldName = sensor_->WorldName();
    world_ = physics::get_world(worldName);

    // read some parameters from the SDF model
    if (sdf->HasElement("robotNamespace"))
      namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      namespace_.clear();

    // compulsory check to see if ROS is correctly initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, "
        "unable to load plugin."
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
        "in the gazebo_ros package)");
      return;
    }

    // create ROS node handle
    nodeHandle_ = std::make_shared<ros::NodeHandle>(namespace_);
    ROS_INFO_NAMED("gazebo_ros_3dlidar",
      "Starting gazebo_ros_3dlidar plugin in namespace: %s",
      namespace_.c_str());

    // read configuration parameters for the plugin
    readParameters();

    // initialize ROS publishers
    initPublishers();

    // init diagnostics engine
    diagnosticUpdater_.setHardwareID("gazebo_ros_3dlidar");

    // reset simulation variables
    Reset();

    // connect to new laser data
    newLaserFrameConnection_ = sensor_->ConnectNewLaserFrame(
      boost::bind(&GazeboRos3dLidar::OnNewLaserFrame,
      this, _1, _2, _3, _4, _5));

    // activate the sensor
    sensor_->SetActive(true);
  }

  void GazeboRos3dLidar::OnNewLaserFrame(const float* scans, unsigned int
      /*width*/, unsigned int /*height*/, unsigned int /*depth*/, const
      std::string& /*format*/) {
    publishPointCloud(scans);
    diagnosticUpdater_.update();
  }

  void GazeboRos3dLidar::readParameters() {
    // robot state
    nodeHandle_->param<std::string>("lidar/topic",
      pointCloudPublisherTopic_, "lidar/point_cloud");
    nodeHandle_->param<std::string>("lidar/frame_id",
      pointCloudPublisherFrameId_, "lidar_link");
    nodeHandle_->param<int>("lidar/queue_size",
      pointCloudPublisherQueueSize_, 100);
    double pointCloudPublisherMinFrequencyRate,
      pointCloudPublisherMaxFrequencyRate;
    nodeHandle_->param<double>("lidar/min_frequency_rate",
      pointCloudPublisherMinFrequencyRate, 0.1);
    nodeHandle_->param<double>("lidar/max_frequency_rate",
      pointCloudPublisherMaxFrequencyRate, 0.1);
#if GAZEBO_MAJOR_VERSION >= 7
    pointCloudPublisherMinFrequency_ = sensor_->UpdateRate() -
      pointCloudPublisherMinFrequencyRate * sensor_->UpdateRate();
    pointCloudPublisherMaxFrequency_ = sensor_->UpdateRate() +
      pointCloudPublisherMaxFrequencyRate * sensor_->UpdateRate();
#else
    pointCloudPublisherMinFrequency_ = sensor_->GetUpdateRate() -
      pointCloudPublisherMinFrequencyRate * sensor_->GetUpdateRate();
    pointCloudPublisherMaxFrequency_ = sensor_->GetUpdateRate() +
      pointCloudPublisherMaxFrequencyRate * sensor_->GetUpdateRate();
#endif
  }

  void GazeboRos3dLidar::initPublishers() {
    // point cloud publisher
    ros::AdvertiseOptions pointCloudPublisherOptions;
    pointCloudPublisherOptions.init<sensor_msgs::PointCloud>(
      pointCloudPublisherTopic_, pointCloudPublisherQueueSize_);
    pointCloudPublisher_ = nodeHandle_->advertise(pointCloudPublisherOptions);
    pointCloudPublisherFrequencyDiagnostic_ =
      std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
      pointCloudPublisherTopic_, diagnosticUpdater_,
      diagnostic_updater::FrequencyStatusParam(
      &pointCloudPublisherMinFrequency_, &pointCloudPublisherMaxFrequency_,
      0.1, 10));
  }

  void GazeboRos3dLidar::publishPointCloud(const float* scans) {
    pointCloudPublisherFrequencyDiagnostic_->tick();
    if (pointCloudPublisher_.getNumSubscribers() == 0)
      return;

    // init message
    auto pointCloud = boost::make_shared<sensor_msgs::PointCloud>();
#if GAZEBO_MAJOR_VERSION >= 8
    pointCloud->header.stamp = ros::Time(world_->SimTime().sec,
                                         world_->SimTime().nsec);
#else
    pointCloud->header.stamp = ros::Time(world_->GetSimTime().sec,
                                         world_->GetSimTime().nsec);
#endif
    pointCloud->header.frame_id = pointCloudPublisherFrameId_;
#if GAZEBO_MAJOR_VERSION >= 7
    const auto numRaysHorizontal = sensor_->RayCount();
    const auto numRaysVertical = sensor_->VerticalRayCount();
#else
    const auto numRaysHorizontal = sensor_->GetRayCount();
    const auto numRaysVertical = sensor_->GetVerticalRayCount();
#endif
    const auto numPoints = numRaysHorizontal * numRaysVertical;
    pointCloud->points.reserve(numPoints);
    pointCloud->channels.resize(1);
    pointCloud->channels[0].name = "intensity";
    pointCloud->channels[0].values.reserve(numPoints);

    // loop over scans and fill point cloud
    for (size_t j = 0; j < static_cast<size_t>(numRaysVertical); ++j) {
      for (size_t i = 0; i < static_cast<size_t>(numRaysHorizontal); ++i) {
        const auto index = j * numRaysHorizontal + i;
        const auto range = scans[index * 3];
        const auto intensity = scans[index * 3 + 1];
        const auto inclination = getInclination(j);
        const auto azimuth = getAzimuth(i);

        geometry_msgs::Point32 point;
        point.x = range * std::cos(inclination) * std::cos(azimuth);
        point.y = range * std::cos(inclination) * std::sin(azimuth);
        point.z = range * std::sin(inclination);
        pointCloud->points.push_back(point);
        pointCloud->channels[0].values.push_back(intensity);
      }
    }
    pointCloudPublisher_.publish(pointCloud);
  }

  GZ_REGISTER_SENSOR_PLUGIN(GazeboRos3dLidar)
}
