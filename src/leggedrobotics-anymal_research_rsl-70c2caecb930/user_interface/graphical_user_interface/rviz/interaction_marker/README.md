## Interaction marker

The *interaction_marker* package implements a plugin-based interactive marker using *pluginlib*.
This increases the flexibility of the interactive marker while reducing the amount of dependencies.

Each plugin can extend the context menu of the interactive marker and bind according function callbacks.


### Running the interaction marker

Run

    roslaunch interaction_marker interaction_marker.launch


### Testing the interaction marker

Run

    roslaunch interaction_marker test.launch


### Creating a plugin

Have a look at the *interaction_marker_plugin_example* package.

For a demonstration, run

    roslaunch interaction_marker_plugin_example test.launch
