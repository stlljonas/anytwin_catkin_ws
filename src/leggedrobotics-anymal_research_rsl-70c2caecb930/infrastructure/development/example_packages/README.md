# Example Packages - Software Development Instructions

## Overview

This repository contains example packages illustrating the software development instructions.

**Keywords:** example, package, template

**Author(s):** Peter Fankhauser, Remo Diethelm, Gabriel Hottiger, Yvain de Viragh

**Maintainer(s):** <a href="mailto:rdiethelm@anybotics.com">Remo Diethelm</a>

**Affiliation:** [ANYbotics AG](https://www.anybotics.com/)

## Coverage

Content of the repository:

- How to set up a repository (
  [README](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator),
  [LICENSE](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator/package.xml),
  [.gitignore](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/.gitignore) ).

Content of the example catkin packages:

- How to set up a Catkin package ([Package](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator)).
- How to separate algorithms and ROS interface ([Algorithm](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator), [ROS Interface](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator_ros)).
- How to define ROS messages and services ([Package](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator_msgs)).
- How to set Catkin and CMake dependencies properly ([package.xml](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator/package.xml), [CMakeLists.txt](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator/CMakeLists.txt)).
- How to enable C++11 ([CMakeLists.txt](average_calculator/CMakeLists.txt)).
- How to improve code quality by using clang and enabling build warnings ([CMakeLists.txt](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator/CMakeLists.txt)).
- How to print output to the console ([Source Code](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator_ros/src/average_calculator_ros/AverageCalculatorRos.cpp)).
- How to document code using Doxygen and publish the compiled documentation. ([Documenting](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator_ros/include/average_calculator_ros/AverageCalculatorRos.hpp), [Published Documentation](https://anymal-research.docs.anymal.com/doxygen/average_calculator_doc/master/index.html)).
- How to write unit tests:
    - Simple unit tests ([Code](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator/test), [CMakeLists.txt](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator/CMakeLists.txt)).
    - ROS node unit tests ([Code](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator_ros/test), [CMakeLists.txt](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator_ros/CMakeLists.txt)).

Missing content:

- How to set up a user interface package using rqt.
- How to set up a python based package.

## Packages

- Average Calculator ([Doxygen](\ref page_average_calculator) / [README](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator) )
- Average Calculator ROS ([Doxygen](\ref page_average_calculator_ros) / [README](https://code.anymal.com/anymal-research/anymal_research/tree/master/infrastructure/development/example_packages/average_calculator_ros) )
