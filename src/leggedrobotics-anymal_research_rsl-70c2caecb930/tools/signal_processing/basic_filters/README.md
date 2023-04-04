\page page_basic_filters Basic Filters

# Basic Filters

## Overview

This package contains implementations of commonly used filters in signal processing such as:
- Binary chattering compensator
- Continuous time transfer function
- Cumulative moving average filter
- Delay
- Discrete time transfer function
- Exponential moving average filter
- Low-pass Butterworth digital filters
- Second order filter
- Moving average filter
- First order derivative
- First order filter
- Rate limiter filter
- Braking rate limiter filter


## Installation

### Install Debian Packages

To install the basic filters package as Debian packages use

    sudo apt install ros-${ROS_DISTRO}-basic-filters

### Building from Source

#### Dependencies

- [Eigen](http://eigen.tuxfamily.org/) (library for linear algebra)
- [Kindr](https://github.com/ANYbotics/kindr) (Kinematics and Dynamics for Robotics)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd ~/catkin_ws/src
    git clone git@code.anymal.com:anymal-research/anymal_research.git
    catkin build basic_filters

### Unit Tests

Run the unit tests with

    catkin run_tests basic_filters

## Usage

Example code computing the velocity given a maximum limit for accelerating and breaking is given below:

```
#include <iostream>

// basic_filters
#include <basic_filters/filters.hpp>

using namespace basic_filters;

int main(int argc, char** argv)
{
  BrakingRateLimiterDouble velocity;

  // Set zero initial velocity
  filteredValue.initialize(0.0);

  // Set max acceleration = 1.0 m/s^2 and breaking deceleration = 10.0 m/s^2
  filteredValue.setParameters(1.0, 10.0);

  // Compute the updated velocity after 0.5s given a desired velocity = 10.0 m/s
  filteredValue.update(10.0, 0.5);

  const double filteredValue = filteredValue.getValue();
  std::cout << "Rate limited velocity after 0.5s is " << filteredValue << std::endl;

  return 0;
}
```
