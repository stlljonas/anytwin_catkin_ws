# soem_interface

## Overview

This software package serves as an interface for one or more ethercat devices running on the same bus. SOEM is used for the lower level ethercat communication

**Authors(s):** Markus Staeuble

## Installation

### Building from Source

To build the cosmo node from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://bitbucket.org/leggedrobotics/soem_interface
	cd ../
	catkin build soem_interface
	
## Classes

#### EthercatSlaveBase
This is an abstract base class for an ethercat slave. The ethercat slave class holds a non owning reference to the EthercatBusBase it is on (bus_). 
EthercatSlaves should stage their tx messages before writing to the ethercat bus in the updateWrite() method using the bus_ writeRxPdo() method.
Similarly, the slaves can retrieve the buffered read messages from the bus with the readTxPdo() method in updateRead().

#### EthercatBusBase
This class represents a physical ethercat bus containing multiple ethercat slaves. 
It manages the slaves on the bus using the methods from soem.  With updateRead() and updatewrite() the staged slave messages are written/read to/from all the slaves on the bus simultaneously. 
EthercatSlaves can be added with addSlave(). After all the slaves have been added the startup() method to actually start the communication of the bus.

#### EthercatBusManagerBase
If multiple buses are connected to the same master then the busses are managed by the EthercatBusManagerBase. 
	
