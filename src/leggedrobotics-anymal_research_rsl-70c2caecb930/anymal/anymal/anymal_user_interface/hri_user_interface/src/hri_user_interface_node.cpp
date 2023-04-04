/*!
* @file     hri_user_interface_node.cpp
* @author   Christian Gehring
* @date     April 16, 2015
* @brief
*/

#include <ros/ros.h>

#include <any_node/any_node.hpp>

#include <hri_user_interface/HRIUserInterface.h>

int main(int argc, char **argv) {
  any_node::Nodewrap<hri_user_interface::HRIUserInterface> node(argc, argv, "hri_user_interface", 1);
  // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the any_node::Node::shutdown() function)
  return static_cast<int>(!node.execute());
}
