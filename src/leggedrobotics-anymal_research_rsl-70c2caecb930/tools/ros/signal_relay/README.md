# Signal Relay
Currently allows one node to trigger a reset of another without requiring any build dependencies.
Sets up a node that routes one-to-many input reset messages to a list of reset topics or services
exposed by other nodes.

Routes are set by a configuration file, and have the following structure:
```
reset_routes:
  <route_name>:
    input: <reset_topic_in>
    output_topics: [<reset_topic_out1>, <reset_topic_out2>, ...]
    output_services: [<reset_service1>, <reset_service2>, ...]
```

The route name can be arbitrary, and is only used to separate routes. See the example
configuration for more details. Nodes can expose reset functionality by publishing/subscribing to a `std_msgs/Empty` topic,
or advertising a `std_srv/Empty` service.
Publishing an Empty message signifies that a node would like to trigger a reset, and the Signal Relay routes this to the
specified subscribers/services that should reset.

**Note**: The Signal Relay currently does not check for cycles in the reset routes graph. Setting an input and output
to the same topic could trigger loops in the graph. The current convetion is to name the topic `.../reset_in` and
`.../reset_out` to prevent this behavior, but is not strictly enforced.

## Running Tests
`catkin run_tests signal_relay`