/*
 * rpsm_node.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Christian Gehring
 */
#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <iostream>

#include "rpsm_npc/RpsmNode.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<rpsm_npc::RpsmNode> node(argc, argv, "rpsm_npc", 1);
  return static_cast<int>(!node.execute());
}
