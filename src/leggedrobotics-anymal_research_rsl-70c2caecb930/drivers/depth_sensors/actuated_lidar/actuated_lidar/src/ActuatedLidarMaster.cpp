// param io
#include <param_io/get_param.hpp>

// sensor_msgs
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

// std_msgs
#include <std_msgs/Bool.h>

// laser_assembler
#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>

// pcl_ros
#include <pcl_ros/transforms.h>

// actuated_lidar
#include "actuated_lidar/ActuatedLidarMaster.hpp"


namespace actuated_lidar {

using namespace param_io;


ActuatedLidarMaster::ActuatedLidarMaster(ros::NodeHandle& nh)
: nh_(nh)
{
  // parameters
  publishPointCloud2_ = param<bool>(nh_, "point_cloud_publisher/publish_point_cloud_2", false);

  // init subscriber
  motionGoalReachedSubscriber_ = nh_.subscribe(
      param<std::string>(nh_, "subscribers/motion_goal_reached/topic", "/default"),
      param<int>(nh_, "subscribers/motion_goal_reached/queue_size", 1),
      &ActuatedLidarMaster::motionGoalReachedSubscriberCallback, this);

  // init publishers
  if (publishPointCloud2_) {
    pointCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        param<std::string>(nh_, "publishers/point_cloud/topic", "/default"),
        param<int>(nh_, "publishers/point_cloud/queue_size", 1),
        param<bool>(nh_, "publishers/point_cloud/latch", false));
  } else {
    pointCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud>(
        param<std::string>(nh_, "publishers/point_cloud/topic", "/default"),
        param<int>(nh_, "publishers/point_cloud/queue_size", 1),
        param<bool>(nh_, "publishers/point_cloud/latch", false));
  }
  setDirectionPublisher_ = nh_.advertise<std_msgs::Bool>(
      param<std::string>(nh_, "publishers/set_direction/topic", "/default"),
      param<int>(nh_, "publishers/set_direction/queue_size", 1),
      param<bool>(nh_, "publishers/set_direction/latch", false));

  // init clients
  if (publishPointCloud2_) {
    assembleScansServiceClient_ = nh_.serviceClient<laser_assembler::AssembleScans2>(
        param<std::string>(nh_, "clients/assemble_scans/service", "/default"));
  } else {
    assembleScansServiceClient_ = nh_.serviceClient<laser_assembler::AssembleScans>(
        param<std::string>(nh_, "clients/assemble_scans/service", "/default"));
  }

  // parameters
  deviceNames_ = param<std::vector<std::string>>(nh_, "device_names", std::vector<std::string>());
  for (unsigned int i = 0; i < deviceNames_.size(); i++)
  {
    deviceMotionGoalReached_[deviceNames_[i]] = false;
  }
  fixedFrameId_ = param<std::string>(nh_, "frames/fixed_frame_id", "default");
  robotFrameId_ = param<std::string>(nh_, "frames/robot_frame_id", "default");
  permanentRotation_ = param<bool>(nh_, "dynamixel/permanent_rotation", false);
}


ActuatedLidarMaster::~ActuatedLidarMaster() {

}


bool ActuatedLidarMaster::assembleScans() {
  if (timeOfLastScanAssembly_.isZero()) {
    timeOfLastScanAssembly_ = ros::Time::now();
  } else {
    if (ros::service::exists(assembleScansServiceClient_.getService(), false)) {
      if (publishPointCloud2_) {
        laser_assembler::AssembleScans2 srv;
        srv.request.begin = timeOfLastScanAssembly_;
        srv.request.end = ros::Time::now();
        timeOfLastScanAssembly_ = srv.request.end;
        if (!assembleScansServiceClient_.call(srv)) {
          ROS_WARN_STREAM_NAMED("ActuatedLidar", "Assemble scans service call failed: " << assembleScansServiceClient_.getService());
          return false;
        } else {
          if (srv.response.cloud.header.frame_id != fixedFrameId_) {
            ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Assemble scans response cloud expected to be in fixed frame (" << fixedFrameId_ << ").");
            return false;
          } else {
            sensor_msgs::PointCloud2 cloudInRobotFrame;
            if (tfListener_.waitForTransform(robotFrameId_, srv.response.cloud.header.frame_id, srv.response.cloud.header.stamp, ros::Duration(2.0))) {
              if (pcl_ros::transformPointCloud(robotFrameId_, srv.response.cloud, cloudInRobotFrame, tfListener_)) {
                pointCloudPublisher_.publish(cloudInRobotFrame);
              } else {
                ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Point cloud could not be transformed from " << srv.response.cloud.header.frame_id << " to " << robotFrameId_ << " frame.");
                return false;
              }
            } else {
              ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Wait for transform timeout for transformation from " << srv.response.cloud.header.frame_id << " to " << robotFrameId_ << " frame.");
              return false;
            }
          }
        }
      } else {
        laser_assembler::AssembleScans srv;
        srv.request.begin = timeOfLastScanAssembly_;
        srv.request.end = ros::Time::now();
        timeOfLastScanAssembly_ = srv.request.end;
        if (!assembleScansServiceClient_.call(srv)) {
          ROS_WARN_STREAM_NAMED("ActuatedLidar", "Assemble scans service call failed: " << assembleScansServiceClient_.getService());
          return false;
        } else {
          if (srv.response.cloud.header.frame_id != fixedFrameId_) {
            ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Assemble scans response cloud expected to be in fixed frame (" << fixedFrameId_ << ").");
            return false;
          } else {
            sensor_msgs::PointCloud cloudInRobotFrame;
            if (tfListener_.waitForTransform(robotFrameId_, srv.response.cloud.header.frame_id, srv.response.cloud.header.stamp, ros::Duration(2.0))) {
              tfListener_.transformPointCloud(robotFrameId_, srv.response.cloud, cloudInRobotFrame);
              if (cloudInRobotFrame.header.frame_id != "") {
                pointCloudPublisher_.publish(cloudInRobotFrame);
              } else {
                ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Point cloud could not be transformed from " << srv.response.cloud.header.frame_id << " to " << robotFrameId_ << " frame.");
                return false;
              }
            } else {
              ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Wait for transform timeout for transformation from " << srv.response.cloud.header.frame_id << " to " << robotFrameId_ << " frame.");
              return false;
            }
          }
        }
      }
    } else {
      ROS_WARN_STREAM_NAMED("ActuatedLidar", "Assemble scans service not available: " << assembleScansServiceClient_.getService());
    }
  }
  return true;
}


void ActuatedLidarMaster::motionGoalReachedSubscriberCallback(const actuated_lidar_msgs::MotionGoalReachedConstPtr& msg)
{
  deviceMotionGoalReached_[msg->device_name] = true;
  
  // check if we can assemble the scans
  for (unsigned int i = 0; i < deviceNames_.size(); i++)
    if (!deviceMotionGoalReached_[deviceNames_[i]])
      return;
  
  // assemble scans and send message to change direction
  std_msgs::Bool rotatePositiveMsg;
  rotatingPositive_ = !rotatingPositive_;
  rotatePositiveMsg.data = rotatingPositive_;
  setDirectionPublisher_.publish(rotatePositiveMsg);
  assembleScans();
  for (unsigned int i = 0; i < deviceNames_.size(); i++)
    deviceMotionGoalReached_[deviceNames_[i]] = false;
}


} // actuated_lidar
