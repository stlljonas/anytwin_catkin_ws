/*
 * RpsmNode.hpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <rpsm_master/RpsmMaster.hpp>

namespace rpsm_lpc {

class RpsmNode : public rpsm_master::RpsmMaster {
 public:
  RpsmNode() = delete;
  RpsmNode(any_node::Node::NodeHandlePtr nh);
  virtual ~RpsmNode();

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent& event) override;

  bool shutdownLpcService(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res);

 protected:
  //! Callback function that will be called by an incoming MAVLINK shutdown notification event.
  virtual void handleMavlinkMasterMavlinkNotification(mavlink_message_t *mavlink_msg);

  bool shutdownLpc();
  void shutdownNpc();
  void shutdownApc();
  ros::ServiceServer shutdownLpcService_;
  ros::ServiceClient shutdownNpcClient_;
  ros::ServiceClient shutdownApcClient_;

private:
  std::string password_;
};

} /* namespace rpsm_lpc */
