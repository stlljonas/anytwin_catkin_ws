# Numerical Optimization

## Overview

This C++ software package provides an interface for numerical problem formulations and wrappers for different numerical solvers.

The software has been tested under ROS Melodic and Ubuntu 18.04.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author(s): Christian Gehring, Stelian Coros, C. Dario Bellicoso**

Contact: Christian Gehring, cgehring@anybotics.com

Affiliations: Autonomous Systems Lab, ETH Zurich, Carnegie Mellon University

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Packages
* **numopt_common:** interfaces for problems and solvers and helper methods
* **numopt_problems:** example problems for unit testing and benchmarking of solvers

### Solvers

* **numopt_neldermead:** Nelder-Mead method

### QP Solvers
* **numopt_pas:** qpOASES solver (parametric active set algorithm)
* **numopt_quadprog:** QuadProg++ solver (active set algorithm)

