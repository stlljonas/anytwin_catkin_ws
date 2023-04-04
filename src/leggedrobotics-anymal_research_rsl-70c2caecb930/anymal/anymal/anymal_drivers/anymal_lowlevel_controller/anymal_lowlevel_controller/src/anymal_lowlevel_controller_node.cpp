// any node
#include "any_node/any_node.hpp"

// anymal lowlevel controller
#include "anymal_lowlevel_controller/AnymalLowLevelController.hpp"

#include <sys/prctl.h>


int main(int argc, char** argv)
{
  // we need to tell the OS explicitly to also create core dumps for this executable. 
  // The OS disables this per default for binaries which have elevated capabilities, see PR_SET_DUMPABLE section of https://man7.org/linux/man-pages/man2/prctl.2.html
  prctl(PR_SET_DUMPABLE, 1);
  any_node::Nodewrap<anymal_lowlevel_controller::AnymalLowLevelController> node(argc, argv, "anymal_lowlevel_controller", 2); // use 2 spinner thread
  return static_cast<int>(!node.execute());
}
