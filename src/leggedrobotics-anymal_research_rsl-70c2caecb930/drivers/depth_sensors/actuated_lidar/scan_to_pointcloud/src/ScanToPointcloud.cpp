// param io
#include <param_io/get_param.hpp>

// scan to pointcloud
#include "scan_to_pointcloud/ScanToPointcloud.hpp"


namespace scan_to_pointcloud {

using namespace param_io;


ScanToPointcloud::ScanToPointcloud(ros::NodeHandle& nh)
: nh_(nh)
{
  scanSubscriber_ = nh_.subscribe(
      param<std::string>(nh_, "subscribers/scan/topic", ""),
      param<int>(nh_, "subscribers/scan/queue_size", 0),
      &ScanToPointcloud::scanCallback, this);

  pointCloudScanPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
      param<std::string>(nh_, "publishers/point_cloud/topic", ""),
      param<int>(nh_, "publishers/point_cloud/queue_size", 0),
      param<bool>(nh_, "publishers/point_cloud/latch", false));

  targetFrameId_ = param<std::string>(nh_, "target_frame_id", "");
}

ScanToPointcloud::~ScanToPointcloud()
{

}

void ScanToPointcloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if(!tfListener_.waitForTransform(
      scan->header.frame_id,
      targetFrameId_,
      scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
      ros::Duration(1.0)))
  {
    ROS_ERROR_STREAM("ScanToPointcloud: Could not get transformation from '" << scan->header.frame_id << "' to '" << targetFrameId_ << "'.");
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud(targetFrameId_, *scan, cloud, tfListener_);
  pointCloudScanPublisher_.publish(cloud);
  return;
}

} // scan_to_pointcloud
