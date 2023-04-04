# README

### Overview

The source code in this repository implements a [roco](https://github.com/anybotics/roco)-based controller
that uses an optimization-based motion generator (see [motion_generation](http://bitbucket.org/leggedrobotics/motion_generation))
and a whole-body controller (see [whole_body_control](http://bitbucket.org/leggedrobotics/whole_body_control))
to generate a wide variety of walking gaits.

**Author(s)**: C. Dario Bellicoso, Fabian Jenelten

Contact: C. Dario Bellicoso, bellicosodario@gmail.com

Affiliation: Robotic Systems Lab, ETH Zurich

### License
The source code is released under a [proprietary license](LICENSE).

### Parameter files

The controller has several parameters/settings that can be tuned by modyfying the appropriate *.xml files in the config folder.
Please refer to the relative parameter file for further information.

### ROS interface

The **anymal_ctrl_dynamic_gaits_ros** package builds on top of the [loco_ros](http://bitbucket.org/leggedrobotics/loco) package to communicate with external ROS nodes using the ROS API.

### References

[1] C. D. Bellicoso, F. Jenelten, C. Gehring and M. Hutter,
"**Dynamic Locomotion Through Online Nonlinear Motion Optimization for Anymalal Robots**,"
in IEEE Robotics and Automation Letters, vol. 3, no. 3, pp. 2261-2268, July 2018.
doi: 10.1109/LRA.2018.2794620, [Link to IEEE Xplore](https://ieeexplore.ieee.org/document/8260889)

[2] C. Dario Bellicoso, F. Jenelten, P. Fankhauser, C. Gehring, J. Hwangbo and M. Hutter,
"**Dynamic locomotion and whole-body control for anymalal robots**,"
in 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, 2017, pp. 3359-3365.
doi: 10.1109/IROS.2017.8206174, [Link to IEEE Xplore](https://ieeexplore.ieee.org/document/8206174)
