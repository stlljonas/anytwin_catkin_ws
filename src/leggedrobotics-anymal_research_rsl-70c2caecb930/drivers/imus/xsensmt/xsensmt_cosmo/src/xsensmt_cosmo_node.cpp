/*
 * xsensmt_cosmo_node.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: Christian Gehring
 */


#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <iostream>

#include "xsensmt_cosmo/XsensmtCosmoNode.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<xsensmt::XsensmtCosmoNode> node(argc, argv, "xsensmt", 1);
  return static_cast<int>(!node.execute());
}
