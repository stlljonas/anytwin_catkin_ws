/*
 * RpsmNode.hpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <any_node/Node.hpp>
#include <std_srvs/Empty.h>

namespace rpsm_npc {

class RpsmNode : public any_node::Node {
 public:
  RpsmNode() = delete;
  RpsmNode(any_node::Node::NodeHandlePtr nh);
  virtual ~RpsmNode();

  bool init() override;
  void cleanup() override;

  bool shutdownNpcService(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res);

 protected:
  void shutdownNpc();

  ros::ServiceServer shutdownNpcServer_;

 private:
  std::string password_;
};

} /* namespace rpsm_npc */
