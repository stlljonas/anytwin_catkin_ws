/*
 * xsensmt_triggering_cosmo_node.cpp
 *
 *  Created on: Sep 24, 2018
 *      Author: jelavice
 */




#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <iostream>

#include "xsensmt_cosmo/XsensmtCosmoNode.hpp"
#include "xsensmt_cosmo/XsensmtTriggeringCosmoNode.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<xsensmt::XsensmtTriggeringCosmoNode> node(argc, argv, "xsensmt", 1);
  return static_cast<int>(!node.execute());
}
