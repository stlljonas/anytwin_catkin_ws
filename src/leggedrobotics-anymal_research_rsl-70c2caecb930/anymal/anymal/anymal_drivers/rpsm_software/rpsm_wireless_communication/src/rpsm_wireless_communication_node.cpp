/*
 * rpsm_wireless_communication_node.cpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Russell Buchanan
 */

// wireless communication
#include "rpsm_wireless_communication/RPSMWirelessCommunication.hpp"

// nodewrap
#include "any_node/Nodewrap.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<rpsm_wireless_communication::RPSMWirelessCommunication> node(argc, argv, "rpsm_wireless_communication", 1);
  ROS_INFO("Node Created");
  return static_cast<int>(!node.execute());
}