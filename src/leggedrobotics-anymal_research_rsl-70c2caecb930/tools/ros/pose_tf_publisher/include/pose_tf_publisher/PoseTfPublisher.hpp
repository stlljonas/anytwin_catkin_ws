/*
 * PoseTfPublisher.hpp
 *
 *  Created on: Jan 22, 2014
 *      Author: Péter Fankhauser, Christian Gehring
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


namespace pose_tf_publisher {


template<typename StampedMessageType>
class get_pose
{
public:
  static geometry_msgs::Pose getPose(const StampedMessageType& message);
};


template<typename StampedMessageType>
class PoseTfPublisher
{
protected:
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber poseSubscriber_;
  tf::TransformBroadcaster transformBroadcaster_;
  ros::Timer publishingTimer_;

  std::string poseTopic_;
  std::string childFrameId_;
  ros::Time stampOfLastMessage_;
  ros::Duration minTimeStep_;
  std::string parentTfPrefix_;
  std::string childTfPrefix_;
  bool publishContinuously_;

  tf::StampedTransform tfTransform_;

public:
  PoseTfPublisher(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle),
    minTimeStep_(0.00001),
    publishContinuously_(false)
  {
    ROS_INFO("Pose TF publisher node started.");
    if (!readParameters())
    {
      ros::requestShutdown();
    }
    poseSubscriber_ = nodeHandle_.subscribe(poseTopic_, 1, &PoseTfPublisher::poseCallback, this);
    if (publishContinuously_)
    {
      publishingTimer_ = nodeHandle_.createTimer(minTimeStep_, &PoseTfPublisher::publishingTimerCallback, this);
    }
  }

  virtual ~PoseTfPublisher()
  {

  }

  bool readParameters()
  {
    if (!nodeHandle_.getParam("pose_topic", poseTopic_))
    {
      ROS_WARN("Pose TF publisher could not get pose topic parameter.");
      return false;
    }
    if (!nodeHandle_.getParam("child_frame_id", childFrameId_))
    {
      ROS_WARN("Pose TF publisher could not get child frame id parameter.");
      return false;
    }
    double maxRate = 10000;
    if (!nodeHandle_.getParam("max_rate", maxRate))
    {
      ROS_WARN("Pose TF publisher could not get max rate parameter.");
      return false;
    }
    minTimeStep_ = ros::Duration(1.0/maxRate);
    nodeHandle_.getParam("parent_tf_prefix", parentTfPrefix_);
    nodeHandle_.getParam("child_tf_prefix", childTfPrefix_);
    nodeHandle_.getParam("publish_continuously", publishContinuously_);
    return true;
  }

  void poseCallback(const StampedMessageType& message)
  {
    if (message.header.stamp == stampOfLastMessage_)
    {
      // received the same message again, not publishing tf
      ROS_DEBUG("Received the same message again.");
    }
    else
    {
      // received a new message
      if (message.header.stamp - stampOfLastMessage_ >= minTimeStep_)
      {
        stampOfLastMessage_ = message.header.stamp;

        poseMsgToTF(get_pose<StampedMessageType>::getPose(message), tfTransform_);

        tfTransform_.frame_id_ = tf::resolve(parentTfPrefix_, message.header.frame_id);
        tfTransform_.stamp_ = message.header.stamp;
        tfTransform_.child_frame_id_ = tf::resolve(childTfPrefix_, childFrameId_);

        if (!publishContinuously_)
        {
          publishTf();
        }
      }
    }
  }

  void publishingTimerCallback(const ros::TimerEvent& event)
  {
    // return if no pose has been received yet
    if (tfTransform_.stamp_.isZero())
    {
      return;
    }

    // update the time stamp before publishing
    tfTransform_.stamp_ = event.current_expected;
    publishTf();
  }

  void publishTf()
  {
    transformBroadcaster_.sendTransform(tfTransform_);
    ROS_DEBUG_STREAM("Published TF for pose at time " << tfTransform_.stamp_.toSec() << ".");
  }
};


template<>
class get_pose<geometry_msgs::PoseStamped>
{
public:
  static geometry_msgs::Pose getPose(const geometry_msgs::PoseStamped& message)
  {
    return message.pose;
  }
};

template<>
class get_pose<geometry_msgs::PoseWithCovarianceStamped>
{
public:
  static geometry_msgs::Pose getPose(const geometry_msgs::PoseWithCovarianceStamped& message)
  {
    return message.pose.pose;
  }
};

} // pose_tf_publisher
