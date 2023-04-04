/*
 * rpsm_node.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Christian Gehring
 */
#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <iostream>

#include "../include/rpsm_apc/RpsmNode.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<rpsm_apc::RpsmNode> node(argc, argv, "rpsm_apc", 1);
  return static_cast<int>(!node.execute());
}
