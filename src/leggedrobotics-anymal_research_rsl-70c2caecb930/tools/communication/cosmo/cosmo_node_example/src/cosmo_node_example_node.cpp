/*!
 * @file     communicator_node_example_node.cpp
 * @author   Philipp Leemann
 * @date     May, 2017
 * @brief
 */

#include "cosmo_node/cosmo_node.hpp"
#include "cosmo_node_example/CommunicatorNodeExample.hpp"

int main(int argc, char** argv) {
  cosmo_node::Nodewrap<cosmo_node_example::CommunicatorNodeExample> node(argc, argv, "cosmo_node_example", 2);  // use 2 spinner thread
  return static_cast<int>(!node.execute());
}
