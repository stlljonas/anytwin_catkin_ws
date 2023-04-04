# ANYmal B

## Overview

This package contains the description, configuration, and launch files for ANYmal B.

## Documentation

The user manual is available on [http://support.anybotics.com](http://support.anybotics.com).

## Usage

The configuration first needs to be loaded to the ROS parameter server. This can be done by calling

    roslaunch anymal_b load_config.launch ARGS...
    
The documentation of the arguments are displayed when the `--ros-args` option is used.

After that, it is possible to run the software using

    roslaunch stack_launcher stack_launcher.launch stack:=STACK_NAME
    
For convenience, frequently used stacks (e.g. OPC) can be launch with

    roslaunch anymal_b opc.launch

Also, there is a convenience launch file for simulation, which includes loading the configuration and starting the software:

    roslaunch anymal_b sim.launch ARGS...
