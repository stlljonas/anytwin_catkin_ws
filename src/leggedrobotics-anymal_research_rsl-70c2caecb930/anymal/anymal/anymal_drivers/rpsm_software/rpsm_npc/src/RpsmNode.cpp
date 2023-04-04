/*
 * RpsmNode.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: gech
 */

#include "rpsm_npc/RpsmNode.hpp"

namespace rpsm_npc {


RpsmNode::RpsmNode(any_node::Node::NodeHandlePtr nh):
  any_node::Node(nh),
  password_()
{}

RpsmNode::~RpsmNode() {
}

bool RpsmNode::init() {
  getNodeHandle().param<std::string>("password", password_, "");
  shutdownNpcServer_ = getNodeHandle().advertiseService("shutdown", &RpsmNode::shutdownNpcService, this);
  return true;
}

void RpsmNode::cleanup() {
  shutdownNpcServer_.shutdown();
}

bool RpsmNode::shutdownNpcService(std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res) {
  ROS_INFO("[rpsm_npc] Received shutdown request from LPC");
  shutdownNpc();
  return true;
}

void RpsmNode::shutdownNpc() {
  std::string systemCall = "echo '" + password_ + "' | sudo -S shutdown -P now";
  system(systemCall.c_str()) == 0;
  return;
}

} /* namespace rpsm_npc */
