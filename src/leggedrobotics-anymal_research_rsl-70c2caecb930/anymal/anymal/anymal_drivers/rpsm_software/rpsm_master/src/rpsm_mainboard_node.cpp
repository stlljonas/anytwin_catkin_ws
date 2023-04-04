/*
 * rpsm_mainboard_node.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Christian Gehring
 */
#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <iostream>

#include "rpsm_master/RpsmMainboard.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<rpsm_master::RpsmMainboard> node(argc, argv, "rpsm_mainboard", 2);
  return static_cast<int>(!node.execute());
}
