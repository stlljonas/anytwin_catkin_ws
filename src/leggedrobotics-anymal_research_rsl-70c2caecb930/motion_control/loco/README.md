# Loco - Locomotion Controller for Legged Robots

This library provides an interface for a locomotion controller for legged
robots. The current version comes along with an implementation for ANYmal.

Support: https://discourse.anymal-research.org/c/anymal-locomotion/

## Modules

The Loco library is built upon several separate modules.

* torso_control: generation of desired body position projected on the ground
* contact_detection: register the detection of contacts between the feet and the ground or an obstacle
* event_detection: check for early/late touchdowns/liftoffs and slip events
* foot_placement_strategy: plan the swing foot trajectory
* gait_patter: swing foot sequence, swing and stance duration, stride duration
* gait_switcher: generates smooth transitions between gaits
* limb_coordinator: decide wether a leg is a support one or not and set control mode to single joints
* mission_control: get external commands from several devices (e.g. joystick)
* motion_control: implementation of a virtual model controller
* state_switcher: state machine that switches state based on events
* terrain_perception: update the control frame and the model of the perceived terrain
* zmp_optimizer: implementation of zmp optimization for body planning

## Dependencies

* Eigen - linear algebra library (eigen.tuxfamily.org)
* kindr - kinematics and dynamics library (https://github.com/anybotics/kindr)
* tinyxml - XML parser
* OOQP - quadratic programming solver (http://pages.cs.wisc.edu/~swright/ooqp/)
* OOQPEI - Eigen interface for OOQP
* robot_utils - (trajectories, logger)
* robotModel - (model for robot StarlETH)

## Installation

### Building the library

Build the library with CMake:
```bash
mkdir build
cd build
cmake ..
make
```

### Building and running tests

GTests are built as soon as the folder gtest exists in the root folder.

Download and use GTest:

```bash
wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
unzip gtest-1.7.0.zip
ln -s gtest-1.7.0 gtest
mkdir build
cd build
cmake .. -DBUILD_TEST=ON
make check
```

### Building documentation

The documentation is generated with Doxygen in the [loco\_doc](loco_doc)
package.

## Acknowledgements

The original authors of the ``loco`` library are Christian Gehring, C. Dario Bellicoso, Peter Fankhauser and Stelian Coros.

### References

* C. Gehring, S. Coros, M. Hutter, M. Bloesch, M. Hoepflinger, R. Siegwart, “Control of Dynamic Gaits for a Anymalal Robot”, IEEE International Conference on Robotics and Automation, 2013.
* C. Gehring, S. Coros, M. Hutter, M. Bloesch, P. Fankhauser, M. A. Hoepflinger, R. Siegwart, “Towards Automatic Discovery of Agile Gaits for Anymalal Robots”, Proc. of the IEEE/RSJ IEEE International Conference on Robotics and Automation (ICRA), 2014.
