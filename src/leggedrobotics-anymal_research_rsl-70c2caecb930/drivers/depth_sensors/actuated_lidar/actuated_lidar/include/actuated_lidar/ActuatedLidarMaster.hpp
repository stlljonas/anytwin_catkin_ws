#pragma once


// c++
#include <string>
#include <map>
#include <vector>

// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

// actuated lidar msgs
#include <actuated_lidar_msgs/MotionGoalReached.h>


namespace actuated_lidar {


class ActuatedLidarMaster
{
protected:
  //! ROS node handle
  ros::NodeHandle& nh_;

  //! ROS motion goal reached subscriber
  ros::Subscriber motionGoalReachedSubscriber_;

  //! ROS set direction publisher
  ros::Publisher setDirectionPublisher_;
  //! ROS point cloud publisher
  ros::Publisher pointCloudPublisher_;


  //! ROS laser assembler service client
  ros::ServiceClient assembleScansServiceClient_;

  //! Use sensor_msgs/PointCloud2
  bool publishPointCloud2_ = true;
  
  //! Names of actuated lidar devices
  std::vector<std::string> deviceNames_;
  //! Current direction of rotation: positive?
  bool rotatingPositive_ = true;
  //! Permanent rotation
  bool permanentRotation_ = true;

  //! tf listener
  tf::TransformListener tfListener_;
  //! Fixed frame id
  std::string fixedFrameId_;
  //! Robot frame id
  std::string robotFrameId_;
  
  //! Last time when the scans have been assembled
  ros::Time timeOfLastScanAssembly_;
  //! Map from device name to "motion goal reached?"
  std::map<std::string, bool> deviceMotionGoalReached_;

public:
  //! Constructor
  ActuatedLidarMaster(ros::NodeHandle& nh);
  //! Destructor
  virtual ~ActuatedLidarMaster();


protected:
  //! Assemble scans
  bool assembleScans();
  //! Motion goal reached subscriber callback
  void motionGoalReachedSubscriberCallback(const actuated_lidar_msgs::MotionGoalReachedConstPtr& msg);
};


} // actuated_lidar

