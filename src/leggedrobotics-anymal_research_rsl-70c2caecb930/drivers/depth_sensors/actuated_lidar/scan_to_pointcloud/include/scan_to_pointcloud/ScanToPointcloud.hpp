#pragma once


// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>


namespace scan_to_pointcloud {


class ScanToPointcloud
{
protected:
  //! ROS node handle
  ros::NodeHandle& nh_;

  //! ROS laser scan subscriber
  ros::Subscriber scanSubscriber_;

  //! ROS point cloud scan publisher
  ros::Publisher pointCloudScanPublisher_;
  
  //! tf listener
  tf::TransformListener tfListener_;
  //! Map frame id
  std::string targetFrameId_;
  
 //! Laser projection
  laser_geometry::LaserProjection projector_;

public:
  ScanToPointcloud(ros::NodeHandle& nh);
  virtual ~ScanToPointcloud();

protected:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};


} // scan_to_pointcloud

