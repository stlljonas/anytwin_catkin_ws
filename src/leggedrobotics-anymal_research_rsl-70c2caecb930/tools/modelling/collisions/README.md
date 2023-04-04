# Collisions

## Overview
**Collisions** is an interface to collisions checking and detection using bullet and fcl.

The software has been tested under ROS Melodic and Ubuntu 18.04.

**Author(s):** Franklin Perry, Dominic Jud
Contact: Dominic Jud, djud@ethz.ch

## Dependencies

- Bullet ```sudo apt-get install libbullet-dev```

### Only for collisions_fcl

- ros-kinetic-interactive-markers ```sudo apt-get install ros-kinetic-interactive-markers```
- FCL, install system-wide from source https://github.com/flexible-collision-library/fcl
    - It requires building ```libccd``` from source https://github.com/danfis/libccd with CMake options ```-DBUILD_SHARED_LIBS=ON -DENABLE_DOUBLE_PRECISION=ON``` and m4 (```sudo apt-get install m4```).
