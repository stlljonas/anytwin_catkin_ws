/*
 * TopicForward.hpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// ros
#include <ros/ros.h>

// param msg io
#include <param_io/get_param.hpp>


namespace topic_forward {


template<typename MsgT>
class TopicForward
{
protected:
  // typedefs
  typedef boost::shared_ptr<MsgT const> MsgTConstPtr;

  // node handle
  ros::NodeHandle& nh_;

  // subscriber
  ros::Subscriber subscriber_;

  // publisher
  ros::Publisher publisher_;

public:
  TopicForward(ros::NodeHandle& nh)
  : nh_(nh)
  {
    ROS_DEBUG_STREAM("Topic forward node started.");
    initializeNode();
  }

  virtual ~TopicForward()
  {

  }

  void initializeNode()
  {
    // subscriber
    subscriber_ = nh_.subscribe(
        param_io::param<std::string>(nh_, "subscribers/input/topic", ""),
        param_io::param<int>(nh_, "subscribers/input/queue_size", 0),
        &TopicForward::inputCb, this);

    // publisher
    publisher_ = nh_.advertise<MsgT>(
        param_io::param<std::string>(nh_, "publishers/output/topic", ""),
        param_io::param<int>(nh_, "publishers/output/queue_size", 0),
        param_io::param<bool>(nh_, "publishers/output/latch", false));
  }

  void inputCb(const MsgTConstPtr& msg)
  {
    ROS_DEBUG_STREAM("Received input message.");

    publisher_.publish(msg);
  }
};


} // topic_forward
