/*
 * xsensmt_triggering_node.cpp
 *
 *  Created on: Sep 17, 2018
 *      Author: jelavice
 */



#include <ros/ros.h>
#include <any_node/Node.hpp>
#include <any_node/Nodewrap.hpp>
#include <xsensmt_ros/XsensMTROSTriggeringInterface.h>

using namespace xsensmt;

int main(int argc, char** argv) {

  any_node::Nodewrap<XsensMTROSTriggeringInterface> node(argc, argv, "xsensmt_node", 2);
  return static_cast<int>(!node.execute());
}

