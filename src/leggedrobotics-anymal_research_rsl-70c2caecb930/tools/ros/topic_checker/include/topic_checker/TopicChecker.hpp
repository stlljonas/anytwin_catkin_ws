/*
 * TopicChecker.hpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// c++
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <XmlRpcException.h>

// any msgs
#include <any_msgs/State.h>
#include <any_msgs/Toggle.h>

// param msg io
#include <param_io/get_param.hpp>


namespace topic_checker {


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
  MsgTConstPtr lastValidMsg_;

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
class TopicChecker
{
protected:
  // typedefs
  typedef boost::shared_ptr<MsgT const> MsgTConstPtr;

  // node handle
  ros::NodeHandle& nodeHandle_;

  // subscribers
  Subscriber<MsgT> input_;
  std::map<std::string, Subscriber<any_msgs::State>> checks_;

  // publishers
  Publisher output_;

  // servers
  Server toggleChecking_;

  // timers
  ros::Timer backupPublishTimer_;

  // checking
  bool checkingIsEnabled_ = true;
  bool checksAreOk_       = false;

  // publishing behavior if checks fail
  bool         backupPublish_               = false; // if true publish a backup message if checks fail
  bool         backupUseLastValidMessage_   = false; // if true use last valid input message as backup, otherwise use message specified below
  bool         backupUseLastValidStamp_     = false; // if true use the stamp of the last valid input message, otherwise the current time
  bool         backupPublishInCallback_     = false; // if true publish in input callback, otherwise in separate timer callback
  double       backupPublishTimerFrequency_ = 0;     // publish timer frequency, only used if backupPublishInCallback_ is false
  MsgTConstPtr backupConstMessage_;                  // constant backup message, only used if backupUseLastValidMessage_ is false


public:
  TopicChecker(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
  {
    ROS_DEBUG_STREAM("Topic checker node started.");
    if (!readParameters())
    {
      ros::requestShutdown();
    }
    initializeNode();
  }

  virtual ~TopicChecker()
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

  bool getBackupMessage(MsgTConstPtr& backupMessage)
  {
    ROS_ERROR_STREAM("getBackupMessage is not implemented for this message type.");
    return false;
  }

  bool readParameters()
  {
    bool success = true;

    // subscribers
    success = success && getParam("subscribers/input/topic",       input_.topic_);
    success = success && getParam("subscribers/input/queue_size",  input_.queueSize_);
    success = success && getParam("subscribers/input/max_timeout", input_.maxTimeout_);
    try
    {
      ParamContainer paramContainers;
      success = success && getParam("subscribers/checks", paramContainers);
      for (int i = 0; i < paramContainers.size(); i++)
      {
        Subscriber<any_msgs::State> newCheck(paramContainers[i]);
        checks_.insert(std::make_pair(newCheck.topic_, newCheck));
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
    success = success && getParam("servers/toggle_checking", toggleChecking_.service_);

    // publishing behavior if checks fail
    success = success && getParam("check_fail_behavior/publish_backup_message", backupPublish_);
    if (backupPublish_)
    {
      // further publishing parameters have to be set
      success = success && getParam("check_fail_behavior/use_last_valid_message", backupUseLastValidMessage_);
      success = success && getParam("check_fail_behavior/use_last_valid_stamp",   backupUseLastValidStamp_);
      success = success && getParam("check_fail_behavior/publish_in_callback",   backupPublishInCallback_);
      if (!backupPublishInCallback_)
      {
        // publish timer frequency has to be set
        success = success && getParam("check_fail_behavior/publish_timer_frequency", backupPublishTimerFrequency_);
      }
      if (!backupUseLastValidMessage_)
      {
        // default message has to be set
        MsgT backupMessage;
        success = success && param_io::getParam(nodeHandle_, "check_fail_behavior/backup_message", backupMessage);
        backupConstMessage_.reset(new MsgT(backupMessage));
      }
    }

    return success;
  }

  void initializeNode()
  {
    // subscribers
    input_.subscriber_ = nodeHandle_.subscribe<MsgT>(input_.topic_,
                                                     input_.queueSize_,
                                                     &TopicChecker::inputCallback,
                                                     this);
    for (auto& check : checks_)
    {
      check.second.subscriber_ = nodeHandle_.subscribe<any_msgs::State>(check.second.topic_,
                                                                        check.second.queueSize_,
                                                                        boost::bind(&TopicChecker::checkCallback, this, _1, check.second.topic_));
    }

    // publishers
    output_.publisher_ = nodeHandle_.advertise<MsgT>(output_.topic_, output_.queueSize_, output_.latch_);

    // servers
    toggleChecking_.server_ = nodeHandle_.advertiseService(toggleChecking_.service_, &TopicChecker::toggleCheckingCallback, this);

    // timers
    if (backupPublish_ && !backupPublishInCallback_)
    {
      // a separate timer has to be created
      backupPublishTimer_ = nodeHandle_.createTimer(ros::Duration(1.0/backupPublishTimerFrequency_), &TopicChecker::publishBackup, this);
    }
  }

  void inputCallback(const MsgTConstPtr& msg)
  {
    ROS_DEBUG_STREAM("Received message on input topic " << input_.topic_ << ".");

    const ros::Time now = ros::Time::now();
    checksAreOk_ = true;

    if (checkingIsEnabled_)
    {
      // check input message
      ROS_DEBUG_STREAM("Checking input ...");
      if (input_.maxTimeout_ > 0)
      {
        if ((now - msg->header.stamp).toSec() > input_.maxTimeout_)
        {
          ROS_WARN_STREAM("Input is too old.");
          checksAreOk_ = false;
        }
      }

      // check all checking messages
      ROS_DEBUG_STREAM("Checking last received checks ...");
      for (auto& check : checks_)
      {
        if (!check.second.lastValidMsg_)
        {
          ROS_WARN_STREAM("Check " << check.first << " has never been received.");
          checksAreOk_ = false;
        }
        else
        {
          if (!check.second.lastValidMsg_->is_ok)
          {
            ROS_WARN_STREAM("Check " << check.first << " is not ok.");
            checksAreOk_ = false;
          }
          if (check.second.maxTimeout_ > 0)
          {
            if ((now - check.second.lastValidMsg_->stamp).toSec() > check.second.maxTimeout_)
            {
              ROS_WARN_STREAM("Check " << check.first << " is too old.");
              checksAreOk_ = false;
            }
          }
        }
      }
    }
    else
    {
      ROS_DEBUG_STREAM("Checking is disabled.");
    }

    if (checksAreOk_)
    {
      // publish this msg and save it as last valid msg
      input_.lastValidMsg_ = msg;
      if (output_.publisher_.getNumSubscribers() > 0)
      {
        output_.publisher_.publish(msg);
      }

      // inform the user about the publication, even if no subscribers were around
      ROS_DEBUG_STREAM("Published input message to topic " << output_.topic_ << ".");
    }
    else if (backupPublish_ && backupPublishInCallback_)
    {
      publishBackup();
    }
  }

  void checkCallback(const any_msgs::StateConstPtr& msg, const std::string& topic)
  {
    ROS_DEBUG_STREAM("Received message on check topic " << topic << ".");
    checks_[topic].lastValidMsg_ = msg;
  }

  bool toggleCheckingCallback(any_msgs::Toggle::Request& req, any_msgs::Toggle::Response& res)
  {
    if (checkingIsEnabled_ == req.enable)
    {
      ROS_DEBUG_STREAM("Checking is already " << (checkingIsEnabled_ ? "enabled." : "disabled."));
    }
    else
    {
      checkingIsEnabled_ = req.enable;
      ROS_DEBUG_STREAM("Checking has been " << (checkingIsEnabled_ ? "enabled." : "disabled."));
    }

    res.success = true;
    return true;
  }

  void publishBackup(const ros::TimerEvent& event = ros::TimerEvent())
  {
    // only publish backup if checks failed
    if (checkingIsEnabled_ && !checksAreOk_)
    {
      // if last valid message is required, check if it ever has been received
      if ((backupUseLastValidMessage_ || backupUseLastValidStamp_) && !input_.lastValidMsg_)
      {
        ROS_ERROR_STREAM("Has never received a valid input message, no backup message is published.");
        return;
      }

      // publish whatever is specified by the parameters
      if (output_.publisher_.getNumSubscribers() > 0)
      {
        MsgT msg;
        // set the message
        msg = backupUseLastValidMessage_ ? *input_.lastValidMsg_ : *backupConstMessage_;
        // set the stamp
        msg.header.stamp = backupUseLastValidStamp_ ? input_.lastValidMsg_->header.stamp : ros::Time::now();
        output_.publisher_.publish(msg);
      }

      // inform the user about the publication, even if no subscribers were around
      ROS_DEBUG_STREAM("Published backup message to topic " << output_.topic_ << ".");
    }
  }
};


} // topic_checker
