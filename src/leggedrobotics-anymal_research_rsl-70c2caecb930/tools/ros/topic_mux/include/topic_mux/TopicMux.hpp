/*
 * TopicMux.hpp
 *
 *  Created on: Nov 20, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// c++
#include <map>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <XmlRpcException.h>

// any msgs
#include <any_msgs/SetTopic.h>


namespace topic_mux {


typedef XmlRpc::XmlRpcValue ParamContainer;


template<typename MsgT>
class Subscriber
{
public:
  typedef boost::shared_ptr<MsgT const> MsgTConstPtr;

  std::string topic_;
  int queueSize_ = 0;
  double maxTimeout_ = 0;
  ros::Subscriber subscriber_;
  MsgTConstPtr lastMsg_;

public:
  Subscriber() {}
  Subscriber(ParamContainer& paramContainer)
  {
    topic_      = static_cast<std::string>(paramContainer["topic"]);
    queueSize_  = static_cast<int>        (paramContainer["queue_size"]);
    maxTimeout_ = static_cast<double>     (paramContainer["max_timeout"]);
  }
};


class Publisher
{
public:
  std::string topic_;
  int queueSize_ = 0;
  bool latch_ = false;
  ros::Publisher publisher_;
};


class Server
{
public:
  std::string service_;
  ros::ServiceServer server_;
};


template<typename MsgT>
class TopicMux
{
protected:
  // typedefs
  typedef boost::shared_ptr<MsgT const> MsgTConstPtr;

  // node handle
  ros::NodeHandle& nodeHandle_;

  // subscribers
  std::string activeInputTopic_;
  std::string defaultInputTopic_;
  std::map<std::string, Subscriber<MsgT>> inputs_;

  // publisher
  Publisher output_;

  // servers
  Server setActiveInput_;
  Server resetActiveInput_;


public:
  TopicMux(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
  {
    ROS_DEBUG_STREAM("Topic mux node started.");
    if (!readParameters())
    {
      ros::requestShutdown();
    }
    initializeNode();
  }

  virtual ~TopicMux()
  {

  }

  template<typename ParamT>
  bool getParam(const std::string& key, ParamT& param) const
  {
    if (!nodeHandle_.getParam(key, param))
    {
      ROS_ERROR_STREAM("Could not acquire parameter " << key << " from server.");
      return false;
    }
    return true;
  }

  bool readParameters()
  {
    bool success = true;

    // subscribers
    // maxTimeout is 0.0 (infinity) for the default input
    Subscriber<MsgT> defaultInput;
    success = success && getParam("subscribers/default_input/topic",      defaultInput.topic_);
    success = success && getParam("subscribers/default_input/queue_size", defaultInput.queueSize_);
    defaultInputTopic_ = defaultInput.topic_;
    activeInputTopic_ = defaultInputTopic_;
    inputs_.insert(std::make_pair(defaultInput.topic_, defaultInput));
    try
    {
      ParamContainer paramContainers;
      success = success && getParam("subscribers/other_inputs", paramContainers);
      for (int i = 0; i < paramContainers.size(); i++)
      {
        Subscriber<MsgT> otherInput(paramContainers[i]);
        inputs_.insert(std::make_pair(otherInput.topic_, otherInput));
      }
    }
    catch (XmlRpc::XmlRpcException& exception)
    {
      ROS_ERROR_STREAM("Caught XmlRpc exception: " << exception.getMessage());
    }

    // publishers
    success = success && getParam("publishers/output/topic",      output_.topic_);
    success = success && getParam("publishers/output/queue_size", output_.queueSize_);
    success = success && getParam("publishers/output/latch",      output_.latch_);

    // servers
    success = success && getParam("servers/set_active_input",   setActiveInput_.service_);
    success = success && getParam("servers/reset_active_input", resetActiveInput_.service_);

    return success;
  }

  void initializeNode()
  {
    // subscribers
    for (auto& input : inputs_)
    {
      input.second.subscriber_ = nodeHandle_.subscribe<MsgT>(input.second.topic_,
                                                             input.second.queueSize_,
                                                             boost::bind(&TopicMux::inputCallback, this, _1, input.second.topic_));
    }

    // publishers
    output_.publisher_ = nodeHandle_.advertise<MsgT>(output_.topic_, output_.queueSize_, output_.latch_);

    // servers
    setActiveInput_.server_   = nodeHandle_.advertiseService(setActiveInput_.service_,   &TopicMux::setActiveInputCallback,   this);
    resetActiveInput_.server_ = nodeHandle_.advertiseService(resetActiveInput_.service_, &TopicMux::resetActiveInputCallback, this);
  }

  void inputCallback(const MsgTConstPtr& msg, const std::string& topic)
  {
    // check if the input topic is active
    if (topic == activeInputTopic_)
    {
      ROS_DEBUG_STREAM("Received message on active input topic " << activeInputTopic_ << ".");

      // check message age
      if (inputs_[activeInputTopic_].maxTimeout_ > 0)
      {
        const double age = (ros::Time::now() - msg->header.stamp).toSec();
        if (age > inputs_[activeInputTopic_].maxTimeout_)
        {
          ROS_WARN_STREAM("Message on active input is too old (" << age << "s), switching to default input topic for safety.");
          setActiveTopic(defaultInputTopic_);
          return;
        }
      }

      // publish message
      if (output_.publisher_.getNumSubscribers() > 0)
      {
        output_.publisher_.publish(msg);
      }

      // inform the user about the publication, even if no subscribers were around
      ROS_DEBUG_STREAM("Published it to topic " << output_.topic_ << ".");
    }
    else
    {
      ROS_DEBUG_STREAM("Received message on inactive input topic " << topic << ".");
    }
  }

  bool setActiveTopic(const std::string& topic)
  {
    // check if the topic is already active
    if (topic == activeInputTopic_)
    {
      ROS_DEBUG_STREAM("Input topic " << topic << " is already active.");
      return true;
    }

    // check if the topic is known
    bool topicIsKnown = false;
    for (const auto& input : inputs_)
    {
      if (input.second.topic_ == topic)
      {
        topicIsKnown = true;
        break;
      }
    }
    if (!topicIsKnown)
    {
      ROS_ERROR_STREAM("Cannot activate input topic " << topic << " since it is unknown.");
      return false;
    }

    // set active topic
    activeInputTopic_ = topic;
    ROS_DEBUG_STREAM("Input topic " << topic << " has been activated.");
    return true;
  }

  bool setActiveInputCallback(any_msgs::SetTopic::Request& req, any_msgs::SetTopic::Response& res)
  {
    return setActiveTopic(req.topic);
  }

  bool resetActiveInputCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    return setActiveTopic(defaultInputTopic_);
  }
};


} // topic_mux
