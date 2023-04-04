#pragma once


// c++
#include <string>

// ros
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>


namespace actuated_lidar {


class ActuatedLidar
{
protected:
  //! ROS node handle
  ros::NodeHandle& nh_;
  
  //! Actuated lidar device name
  std::string deviceName_;

  //! ROS joint state subscriber
  ros::Subscriber jointStateSubscriber_;
  //! ROS set direction subscriber
  ros::Subscriber setDirectionSubscriber_;
  
  //! ROS motion goal reached publisher
  ros::Publisher motionGoalReachedPublisher_;

  //! ROS shutdown motor service server
  ros::ServiceServer shutdownMotorServiceServer_;

  //! ROS set moving speed service client
  ros::ServiceClient setMovingSpeedServiceClient_;
  //! ROS set goal position service client
  ros::ServiceClient setGoalPositionServiceClient_;
  //! ROS set angle limits service client
  ros::ServiceClient setAngleLimitsServiceClient_;
  //! ROS set torque enable service client
  ros::ServiceClient setTorqueEnableServiceClient_;

  //! Velocity of the servo
  double movingSpeed_ = 0;
  //! Minimum angle of the servo
  double minAngle_ = 0;
  //! Maximum angle of the servo
  double maxAngle_ = 0;
  //! Default angle of the servo
  double defaultAngle_ = 0;
  //! Angle safety margin of the servo
  double angleSafetyMargin_ = 0;
  //! Current angle of the servo
  double currentAngle_ = 0;

  //! If a bagfile (containing dynamixel joint state and hokuyo scan) is played, this node does not communicate with this hardware
  bool bagfileIsPlayed_ = false;
  //! Servo mode has been set
  bool modeHasBeenSet_ = false;
  //! Servo in wheel mode
  bool wheelMode_ = false;
  //! Servo is moving
  bool isMoving_ = false;
  //! Rotates positive
  bool rotatesPositively_ = true;
  //! Permanent rotation
  bool permanentRotation_ = true;


public:
  //! Constructor
  ActuatedLidar(ros::NodeHandle& nh);
  //! Destructor
  virtual ~ActuatedLidar();


protected:
  //! Set the dynamixel mode
  bool setMode();
  //! Wrap an angle into [0, 2pi)
  static double wrapAngle(double angle);
  //! Checks if the motion goal has been reached
  bool motionGoalHasBeenReached() const;
  //! Set a new motion goal
  bool setRotation(bool positive);
  //! Joint state subscriber callback
  void jointStateSubscriberCallback(const sensor_msgs::JointStateConstPtr& msg);
  //! Set direction subscriber callback
  void setDirectionSubscriberCallback(const std_msgs::BoolConstPtr& msg);
  //! Shutdown motor service callback
  bool shutdownMotor(std_srvs::Empty::Request& req,
                     std_srvs::Empty::Response& res);
};


} // actuated_lidar

