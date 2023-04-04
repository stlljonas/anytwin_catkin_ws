/*
 * RpsmNode.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: gech
 */

#include "rpsm_apc/RpsmNode.hpp"

namespace rpsm_apc {


RpsmNode::RpsmNode(any_node::Node::NodeHandlePtr nh):
  any_node::Node(nh),
  password_()
{}


RpsmNode::~RpsmNode() {
}


bool RpsmNode::init() {
  getNodeHandle().param<std::string>("password", password_, "");
  shutdownApcServer_ = getNodeHandle().advertiseService("shutdown", &RpsmNode::shutdownApcService, this);
}


void RpsmNode::cleanup() {
  shutdownApcServer_.shutdown();
}


bool RpsmNode::shutdownApcService(std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res)
{
  shutdownApc();
  return true;
}


void RpsmNode::shutdownApc() {
  std::string systemCall = "echo '" + password_ + "' | sudo -S shutdown -P now";
  system(systemCall.c_str()) == 0;
  return;
}

} /* namespace rpsm_apc */
