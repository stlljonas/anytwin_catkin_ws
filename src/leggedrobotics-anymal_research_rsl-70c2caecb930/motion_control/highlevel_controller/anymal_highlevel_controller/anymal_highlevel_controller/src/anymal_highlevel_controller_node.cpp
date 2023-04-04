/*!
 * @file    locomotion_controller_node.cpp
 * @author  Christian Gehring
 * @date    Oct, 2014
 */
#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <iostream>

#include "anymal_highlevel_controller/AnymalHighLevelController.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<anymal_highlevel_controller::AnymalHighLevelController> node(argc, argv, "anymal_highlevel_controller", 2);
  return static_cast<int>(!node.execute());
}
