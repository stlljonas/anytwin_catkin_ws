/** \file xsensmt_node.cpp
    \brief This file is the ROS node for Xsens MT devices.
  */

#include <ros/ros.h>
#include <any_node/Node.hpp>
#include <any_node/Nodewrap.hpp>
#include <xsensmt_ros/XsensMTROSInterface.h>

using namespace xsensmt;

int main(int argc, char** argv) {

  any_node::Nodewrap<XsensMTROSInterface> node(argc, argv, "xsensmt_node", 2);
  return static_cast<int>(!node.execute());
}
