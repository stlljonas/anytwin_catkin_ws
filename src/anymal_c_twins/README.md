# ANYmal C

## Overview

This package contains the description, configuration, and launch files for ANYmal C.

## Documentation

The user manual is available on [http://support.anybotics.com](http://support.anybotics.com).

## Usage

The configuration first needs to be loaded to the ROS parameter server. This can be done by calling

    roslaunch anymal_c load_config.launch ARGS...
    
The documentation of the arguments are displayed when the `--ros-args` option is used.

After that, it is possible to run the software using

    roslaunch stack_launcher stack_launcher.launch stack:=STACK_NAME
    
For convenience, frequently used stacks (e.g. OPC) can be launch with

    roslaunch anymal_c opc.launch

Also, there is a convenience launch file for simulation, which includes loading the configuration and starting the software:

    roslaunch anymal_c sim.launch ARGS...
