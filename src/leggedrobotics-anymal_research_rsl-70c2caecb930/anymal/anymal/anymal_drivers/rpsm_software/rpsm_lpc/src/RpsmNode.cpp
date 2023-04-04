/*
 * RpsmNode.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: gech
 */

#include "rpsm_lpc/RpsmNode.hpp"

#include <thread>


namespace rpsm_lpc {


RpsmNode::RpsmNode(any_node::Node::NodeHandlePtr nh)
: RpsmMaster(nh),
  password_()
{}

RpsmNode::~RpsmNode() {}

bool RpsmNode::init() {
  bool success = true;
  success = success && RpsmMaster::init();
  success = success && getNodeHandle().param<std::string>("password", password_, "");
  shutdownLpcService_ = getNodeHandle().advertiseService("shutdown", &RpsmNode::shutdownLpcService, this);
  shutdownNpcClient_ = getNodeHandle().serviceClient<std_srvs::Empty>("/rpsm_npc/shutdown");
  shutdownApcClient_ = getNodeHandle().serviceClient<std_srvs::Empty>("/rpsm_apc/shutdown");
  success = success && addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0),
            static_cast<bool (RpsmNode::*)(const any_worker::WorkerEvent& event)>(&RpsmNode::update),
            this, 59);
  return success;
}

void RpsmNode::cleanup() {
  shutdownLpcService_.shutdown();
  shutdownNpcClient_.shutdown();
  shutdownApcClient_.shutdown();
  RpsmMaster::cleanup();
}

bool RpsmNode::update(const any_worker::WorkerEvent& event) {
  return RpsmMaster::update(event);
}

bool RpsmNode::shutdownLpcService(std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res) {
  shutdownEventPublisher_.publish(std_msgs::Empty());
  std::thread apcShutdownThread(&RpsmNode::shutdownApc, this);
  apcShutdownThread.detach();
  std::thread npcShutdownThread(&RpsmNode::shutdownNpc, this);
  npcShutdownThread.detach();
  return shutdownLpc();
}

void RpsmNode::handleMavlinkMasterMavlinkNotification(mavlink_message_t *mavlink_msg) {
  mavlink_master_mavlink_notification_t dec_msg;
  mavlink_msg_master_mavlink_notification_decode(mavlink_msg, &dec_msg);
  ROS_DEBUG_STREAM("RpsmNode:: << [Code: " << dec_msg.code
                  << ", Level: " << (int) dec_msg.level
                  << ", Name: " << dec_msg.name
                  << ": " << dec_msg.description << "]");
  switch (dec_msg.level) {
    case 5:
    {
      shutdownEventPublisher_.publish(std_msgs::Empty());
      std::thread apcShutdownThread(&RpsmNode::shutdownApc, this);
      apcShutdownThread.detach();
      std::thread npcShutdownThread(&RpsmNode::shutdownNpc, this);
      npcShutdownThread.detach();
      shutdownLpc();
      break;
    }
  }
}

bool RpsmNode::shutdownLpc() {
  std::string systemCall = "echo '" + password_ + "' | sudo -S shutdown -P now";
  return system(systemCall.c_str());;
}


void RpsmNode::shutdownNpc() {
  if (shutdownNpcClient_.exists()) {
    std_srvs::Empty npcShutdown;
    ROS_INFO("[rpsm_lpc] Request rpsm_npc to shutdown.");
    shutdownNpcClient_.call(npcShutdown);
  }
  return;
}


void RpsmNode::shutdownApc() {
  if (shutdownApcClient_.exists()) {
    std_srvs::Empty apcShutdown;
    ROS_INFO("[rpsm_lpc] Request rpsm_apc to shutdown.");
    shutdownApcClient_.call(apcShutdown);
  }
  return;
}


} /* namespace rpsm_lpc */
