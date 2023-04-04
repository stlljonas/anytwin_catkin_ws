// param io
#include <param_io/get_param.hpp>

// dynamixel ros msgs
#include <dynamixel_ros_msgs/SetAngleLimits.h>
#include <dynamixel_ros_msgs/SetGoalPosition.h>
#include <dynamixel_ros_msgs/SetMovingSpeed.h>
#include <dynamixel_ros_msgs/SetTorqueEnable.h>

// actuated lidar msgs
#include <actuated_lidar_msgs/MotionGoalReached.h>

// actuated lidar
#include "actuated_lidar/ActuatedLidar.hpp"


namespace actuated_lidar {

using namespace param_io;


ActuatedLidar::ActuatedLidar(ros::NodeHandle& nh)
: nh_(nh)
{
  // init subscribers
  jointStateSubscriber_ = nh_.subscribe(
      param<std::string>(nh_, "subscribers/joint_state/topic", "/default"),
      param<int>(nh_, "subscribers/joint_state/queue_size", 1),
      &ActuatedLidar::jointStateSubscriberCallback, this);
  setDirectionSubscriber_ = nh_.subscribe(
      param<std::string>(nh_, "subscribers/set_direction/topic", "/default"),
      param<int>(nh_, "subscribers/set_direction/queue_size", 1),
      &ActuatedLidar::setDirectionSubscriberCallback, this);
  
  // init publishers
  motionGoalReachedPublisher_ = nh_.advertise<actuated_lidar_msgs::MotionGoalReached>(
      param<std::string>(nh_, "publishers/motion_goal_reached/topic", "/default"),
      param<int>(nh_, "publishers/motion_goal_reached/queue_size", 1),
      param<bool>(nh_, "publishers/motion_goal_reached/latch", false));

  // init servers
  shutdownMotorServiceServer_ = nh_.advertiseService(
      param<std::string>(nh_, "servers/shutdown_motor/service", "/default"),
      &ActuatedLidar::shutdownMotor, this);
  
  // init clients
  setMovingSpeedServiceClient_ = nh_.serviceClient<dynamixel_ros_msgs::SetMovingSpeed>(
      param<std::string>(nh_, "clients/set_moving_speed/service", "/default"));
  setGoalPositionServiceClient_ = nh_.serviceClient<dynamixel_ros_msgs::SetGoalPosition>(
      param<std::string>(nh_, "clients/set_goal_position/service", "/default"));
  setAngleLimitsServiceClient_ = nh_.serviceClient<dynamixel_ros_msgs::SetAngleLimits>(
      param<std::string>(nh_, "clients/set_angle_limits/service", "/default"));
  setTorqueEnableServiceClient_ = nh_.serviceClient<dynamixel_ros_msgs::SetTorqueEnable>(
      param<std::string>(nh_, "clients/set_torque_enable/service", "/default"));

  // parameters
  deviceName_ = param<std::string>(nh_, "device_name", "default");
  bagfileIsPlayed_ = param<bool>(nh_, "bagfile_is_played", false);
  wheelMode_ = param<bool>(nh_, "dynamixel/wheel_mode", false);
  permanentRotation_ = param<bool>(nh_, "dynamixel/permanent_rotation", false);
  movingSpeed_ = param<double>(nh_, "dynamixel/moving_speed", 0.0);
  minAngle_ = param<double>(nh_, "dynamixel/min_angle", 0.0);
  maxAngle_ = param<double>(nh_, "dynamixel/max_angle", 0.0);
  defaultAngle_ = param<double>(nh_, "dynamixel/default_angle", 0.0);
  angleSafetyMargin_ = param<double>(nh_, "dynamixel/angle_safety_margin", 0.0);
  if (permanentRotation_ && !wheelMode_) {
    ROS_WARN_STREAM_NAMED("ActuatedLidar", "Permanent rotation requires wheel mode, enabling it.");
    wheelMode_ = true;
  }
  if (minAngle_ >= maxAngle_) {
    ROS_WARN_STREAM_NAMED("ActuatedLidar", "Minimum angle should be smaller than maximum angle (min = " << minAngle_ << ", max = " << maxAngle_ << "). Setting default values.");
    minAngle_ = 0.5 * M_PI;
    maxAngle_ = 1.5 * M_PI;
  }
  defaultAngle_ = wrapAngle(defaultAngle_);
  if (angleSafetyMargin_ < 0.05) {
    ROS_WARN_STREAM_NAMED("ActuatedLidar", "Angular safety margin must be at least 0.05. Overwriting it.");
    angleSafetyMargin_ = 0.05;
  }
}


ActuatedLidar::~ActuatedLidar() {

}


bool ActuatedLidar::setMode() {
  if (!bagfileIsPlayed_) {
    if (ros::service::exists(setAngleLimitsServiceClient_.getService(), false)) {
      dynamixel_ros_msgs::SetAngleLimits srv;
      if (wheelMode_) {
        srv.request.cw_angle_limit = 0.0;
        srv.request.ccw_angle_limit = 0.0;
      } else {
        srv.request.cw_angle_limit  = std::max(minAngle_ - angleSafetyMargin_, 0.0);
        srv.request.ccw_angle_limit = std::min(maxAngle_ + angleSafetyMargin_, 2.0*M_PI);
      }
      if (!setAngleLimitsServiceClient_.call(srv)) {
        ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set angle limits service call failed: " << setAngleLimitsServiceClient_.getService());
        return false;
      } else if (!srv.response.response) {
        ROS_WARN_STREAM_NAMED("ActuatedLidar", srv.response.message);
        return false;
      } else {
        modeHasBeenSet_ = true;
        if (wheelMode_) {
          ROS_INFO_STREAM_NAMED("ActuatedLidar", "Wheel mode is activated");
        } else {
          ROS_INFO_STREAM_NAMED("ActuatedLidar", "Wheel mode is deactivated");
        }
      }
    } else {
      ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set angle limits service not available: " << setAngleLimitsServiceClient_.getService());
      return false;
    }
  }
  return true;
}


double ActuatedLidar::wrapAngle(double angle)
{
  while (angle < 0.0)
    angle += 2.0*M_PI;
  while (angle >= 2.0*M_PI)
    angle -= 2.0*M_PI;
  return angle;
}


bool ActuatedLidar::motionGoalHasBeenReached() const {
  if (permanentRotation_)
  {
    static double lastAngle = 0.0;
    const bool reached =
        (((currentAngle_ > defaultAngle_ == rotatesPositively_) &&
          (lastAngle <= defaultAngle_ == rotatesPositively_)) ||
         ((wrapAngle(currentAngle_ - M_PI) > defaultAngle_ == rotatesPositively_) &&
          (wrapAngle(lastAngle - M_PI) <= defaultAngle_ == rotatesPositively_)));
    lastAngle = currentAngle_;
    return reached;
  }
  else
  {
    if (rotatesPositively_) {
      return currentAngle_ > maxAngle_;
    } else {
      return currentAngle_ < minAngle_;
    }
  }
}


bool ActuatedLidar::setRotation(bool positive) {
  if (!bagfileIsPlayed_) {
    if (permanentRotation_) {
      if (wheelMode_) {
        if (ros::service::exists(setMovingSpeedServiceClient_.getService(), false)) {
          dynamixel_ros_msgs::SetMovingSpeed srv;
          double goalAngle = positive ? defaultAngle_ : wrapAngle(defaultAngle_ + M_PI);
          //const double movingSpeed = wrapAngle(goalAngle - currentAngle_) / M_PI * movingSpeed_;
          const double movingSpeed = movingSpeed_;
          srv.request.moving_speed = movingSpeed;
          srv.request.torque_limit = 1.0;
          if (!setMovingSpeedServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set moving speed service call failed: " << setMovingSpeedServiceClient_.getService());
            return false;
          } else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("ActuatedLidar", srv.response.message);
            return false;
          } else {
            ROS_DEBUG_STREAM_NAMED("ActuatedLidar", "Set moving speed service called successfully.");
          }
        } else {
          ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set moving speed service not available: " << setMovingSpeedServiceClient_.getService());
          return false;
        }
      } else {
        ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Permanent rotation requires wheel mode.");
        return false;
      }
    } else {
      if (wheelMode_) {
        if (ros::service::exists(setMovingSpeedServiceClient_.getService(), false)) {
          dynamixel_ros_msgs::SetMovingSpeed srv;
          srv.request.moving_speed = positive? movingSpeed_ : -movingSpeed_;
          srv.request.torque_limit = 1.0;
          if (!setMovingSpeedServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set moving speed service call failed: " << setMovingSpeedServiceClient_.getService());
            return false;
          } else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("ActuatedLidar", srv.response.message);
            return false;
          } else {
            ROS_DEBUG_STREAM_NAMED("ActuatedLidar", "Set moving speed service called successfully.");
          }
        } else {
          ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set moving speed service not available: " << setMovingSpeedServiceClient_.getService());
          return false;
        }
      } else {
        if (ros::service::exists(setGoalPositionServiceClient_.getService(), false)) {
          dynamixel_ros_msgs::SetGoalPosition srv;
          srv.request.goal_position = positive? maxAngle_ + 0.5*angleSafetyMargin_ : minAngle_ - 0.5*angleSafetyMargin_;
          srv.request.moving_speed = positive? movingSpeed_ : -movingSpeed_;
          srv.request.torque_limit = 1.0;
          if (!setGoalPositionServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set goal position service call failed: " << setGoalPositionServiceClient_.getService());
            return false;
          } else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("ActuatedLidar", srv.response.message);
            return false;
          } else {
            ROS_DEBUG_STREAM_NAMED("ActuatedLidar", "Set goal position service called successfully.");
          }
        } else {
          ROS_WARN_STREAM_NAMED("ActuatedLidar", "Set goal position service not available: " << setGoalPositionServiceClient_.getService());
          return false;
        }
      }
    }
  }
  isMoving_ = true;
  rotatesPositively_ = positive;
  return true;
}


void ActuatedLidar::jointStateSubscriberCallback(const sensor_msgs::JointStateConstPtr& msg) {
  currentAngle_ = msg->position[0];
  if (!modeHasBeenSet_) {
    if (!setMode()) {
      return;
    }
  }
  if (!isMoving_) {
    setRotation(true);
    return;
  }
  if (motionGoalHasBeenReached()) {
    actuated_lidar_msgs::MotionGoalReached mgrMsg;
    mgrMsg.stamp = ros::Time::now();
    mgrMsg.device_name = deviceName_;
    motionGoalReachedPublisher_.publish(mgrMsg);
  }
}


void ActuatedLidar::setDirectionSubscriberCallback(const std_msgs::BoolConstPtr& msg)
{
  setRotation(msg->data);
}


bool ActuatedLidar::shutdownMotor(std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res)
{
  // shutdown joint state subscriber
  jointStateSubscriber_.shutdown();
  // disable torque
  if (!setTorqueEnableServiceClient_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Service server '" << setTorqueEnableServiceClient_.getService() << "' does not exist.");
    return false;
  }
  dynamixel_ros_msgs::SetTorqueEnable setTorqueEnable;
  setTorqueEnable.request.enable_torque = false;
  if (!setTorqueEnableServiceClient_.call(setTorqueEnable))
  {
    ROS_ERROR_STREAM_NAMED("ActuatedLidar", "Service server '" << setTorqueEnableServiceClient_.getService() << "' returned false.");
    return false;
  }
  return true;
}


} // actuated_lidar
