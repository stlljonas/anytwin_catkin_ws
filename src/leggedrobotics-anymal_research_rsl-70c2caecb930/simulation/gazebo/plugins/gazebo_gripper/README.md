# gazebo_gripper

## Overview

Collection of useful tools for gazebo, that are not plugins themselves (but may be used in plugins).

## Tools

### SimGripper

This approximates a gripper by rigidly attaching "grasped" objects to the robot.

To use it, one needs to specify a collisionLink and a palmLink. When the gripper attempts to grasp something, the SimGripper will check if another object is in collision with the collisionLink. If there is, a rigid joint will be created between the palmLink and the object. When the object is released, this joint is simply detached.

In your plugin, create the SimGripper and specify collision and palm links. When the plugin is attempting to "grasp" something (and/or when the gripper or hand is closing), call attachGrasp(). When the plugin is attempting to release a grasp object (and/or when the gripper or hand is opening), call releaseGrasp().

Important Note: the collisionLink can be any link in the URDF; the gazebo model will be searched for collision objects associated with that link. However, the palmLink must be a link in the gazebo model, which means that links that are children of fixed joints in the URDF cannot be used as a palmLink.
