# topic_mux

## Overview

Topic_mux is a topic multiplexer.
It has several inputs (*default_input* and *other_inputs*) whereof it forwards only one to the output.
Two services are provided to set the active input topic or reset it to default.
When a message is received on a non-default channel, its age is being checked.
If the age is above *max_timeout*, the default channel will be activated automatically for safety.
A value of 0.0 means infinity and the node will accept a message of arbitrary age.

## input and output

Input and output are message topics of the same arbitrary type.
To add support for a new type, please create a copy of *src/pose_stamped_mux_node.cpp* and name it appropriately.

## servers

The user can set the active input topic with *set_active_input* of type *any_msgs::SetTopic*.
*reset_active_input* of type *std_srvs::Empty* will reset the active input topic to default.

## parameters

To get an overview of all topics, servers and settings, please check out the example in *config/example.yaml*.
