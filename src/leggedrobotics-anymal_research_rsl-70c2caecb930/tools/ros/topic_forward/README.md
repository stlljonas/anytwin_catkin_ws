# topic_forward

## Overview

Templated node which forwards a message. Useful in situations where multiple nodes are subscribing on the same topic which has to be sent over WiFi.
In this case, *topic_forward* can subscribe to the topic and distribute the messages to all other nodes lowering the required bandwidth.

## input and output

Input and output is a message topic of arbitrary type.
Please check the *tf* example on how to implement your own type.

