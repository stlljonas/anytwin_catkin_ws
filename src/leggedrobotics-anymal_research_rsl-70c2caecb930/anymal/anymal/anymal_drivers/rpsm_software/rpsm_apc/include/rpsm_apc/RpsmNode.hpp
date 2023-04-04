/*
 * RpsmNode.hpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <any_node/Node.hpp>
#include <std_srvs/Empty.h>

namespace rpsm_apc {

class RpsmNode : public any_node::Node {
 public:
  RpsmNode() = delete;
  RpsmNode(any_node::Node::NodeHandlePtr nh);
  virtual ~RpsmNode();

  bool init() override;
  void cleanup() override;

  bool shutdownApcService(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res);

 protected:
  void shutdownApc();

  ros::ServiceServer shutdownApcServer_;

 private:
  std::string password_;
};

} /* namespace rpsm_apc */
