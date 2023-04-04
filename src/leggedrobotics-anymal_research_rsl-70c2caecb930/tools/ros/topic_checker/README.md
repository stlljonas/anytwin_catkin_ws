# topic_checker

## Overview

The purpose of this node is to forward a message if a list of checks is successful.
If one of the checks fails, a backup message can be sent.
This backup message can be the last message, where all the checks were still fine, or a constant user defined message.
The checking and publishing is done directly in the input callback function to reduce delay.
The publication of the backup message can however be done in a seperate timer, if a constant rate is desired.

## input and output

Input and output is a message topic of arbitrary type.
To add support for a new type, please create a copy of *src/pose_stamped_checker_node.cpp* and name it appropriately.

## checks

The node is checking if the age of all the input messages and checks is below their *max_timeout* specified in the parameters.
A value of 0.0 means infinity and the node will accept a message of arbitrary age.
Furthermore, the check succeeds only if all check messages (of type *any_msgs::State*) contain *is_ok*.

## servers

The user can activate and deactivate the checking with *toggle_checking* of type *any_msgs::Toggle*.

## parameters

To get an overview of all topics, servers and settings, please check out the example in *config/example.yaml*.
