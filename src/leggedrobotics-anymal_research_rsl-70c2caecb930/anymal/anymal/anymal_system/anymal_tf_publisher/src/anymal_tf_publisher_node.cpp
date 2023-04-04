/*!
* @file     anymal_tf_publisher_node.cpp
* @author   Christian Gehring
* @date     Sep 21, 2015
* @brief
*/

#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <anymal_tf_publisher/AnymalTfPublisher.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<anymal_tf_publisher::AnymalTfPublisher> node(argc, argv, "anymal_tf_publisher", 1);
  return static_cast<int>(!node.execute());
}


