# Free Gait Implementation and Actions for ANYmal

## Build

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Dependencies

* [Free Gait](https://github.com/anybotics/free_gait)

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/leggedrobotics/free_gait_anymal/issues).

## Unit Tests

	catkin build free_gait_anymal_common --no-deps --verbose --catkin-make-args run_tests
	rostest free_gait_anymal_common free_gait_anymal_common.test -t

Note: To see the results of the optimization steps in RViz, set the fixed frame to `odom` and the RobotModel TF Prefix to `final` or `preview`.
