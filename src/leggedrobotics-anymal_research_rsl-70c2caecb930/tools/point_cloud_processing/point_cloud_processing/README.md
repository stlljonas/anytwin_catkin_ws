# Point cloud processor

This packages provides a simple ROS node for point cloud processing on the fly.

It can receive multiple point cloud streams and:

* Apply a chain of libPointMatcher filters to them.
* Merge input clouds, with one of the input streams acting as a *master*.
* Apply filters before and after merging.

## Installation

### Building from Source

#### Dependencies

- Robot Operating System (ROS)
- libpointmatcher
- pointmatcher-ros
- param_io

#### Building

From source

	cd catkin_workspace/src
	git clone git@bitbucket.org:leggedrobotics/point_cloud_processing.git
	cd ../
	catkin build point_cloud_processor

## Usage

Launch node with

	roslaunch point_cloud_processor example_node.launch

Launch nodelet with

	roslaunch point_cloud_processor example_nodelet.launch

## Launch files

* **example_node.launch:** Launches a node that listens for incoming point clouds.
* **example_nodelet.launch:** Launches a nodelet that listens for incoming point clouds.

## Nodes/nodelets

- point_cloud_processor

#### Subscribed Topics

Input topics declared in ```point_cloud_processor/config/example_sensor_streams.yaml```


#### Published Topics

Output topics declared in ```point_cloud_processor/config/example_sensor_streams.yaml```


## Parameters

### Sensor streams declaration
The location of the config files must be specified by means of the parameter **config_files_folder**

A time period (in seconds) for checking if all filtered topics are actively being requested by other nodes can be specified with the parameter **subscribers_check_period**

The following parameters can be used to define a ```source_stream```:

- **id**: id of stream. _Type: int._
- **source_name**: name of stream. _Type: string._
- **source_type**: type of stream (lidar, depth camera, etc). Type: string.
- **queue_size**: queue_size of cloud subscriber. _Type: int. Default: 1._
- **latch**: whether the output topic will be latched. _Type: bool. Default: false._
- **input_topic**: name of input cloud topic. _Type: string._
- **output_topic**: name of output cloud topic. _Type: string._
- **filter**: whether received clouds will be filtered. If false, the node doesn't publish anything, just stores the cloud. _Type: bool. Default: false._
- **filters_config**: name of the filter parameters file. _Type: string._
- **self_filter_config**: (Optional) name of the self filtering parameter file. _Type: string._
- **robot_description**: name of the robot description to use. Only required if self filtering is applied. _Type: string._

The following parameters can be used to define a ```merged_stream```:

- **source_name**: name of stream. _Type: string._
- **master**: id of master stream. _Type: int._
- **sources**: ids of streams to be combined. _Type: array of int._
- **decimation**: skip n consecutive master clouds before merging. _Type: int._
- **target_frame**: frame ID of the output point cloud.. _Type: string._
- **keep_intensity**: whether intensity data from the master sensor should be kept in the merged cloud.. _Type: bool. Default: false_
- **max_time_offset**: max time offset between clouds to merge. _Type: double. Default: 0.5s_
- **wait_time_tf**: blocking time when waiting for tf. _Type: string. Default: 0.1s_
- **use_fixed_frame_tf**: whether a fixed frame will be used for finding the tf between two sensor frames. _Type: bool. Default: false._
- **fixed_frame_tf**: name of fixed frame. _Type: string._
- **latch**: whether the output topic will be latched. _Type: bool. Default: false._
- **output_topic**: name of output cloud topic. _Type: string._
- **filter**: whether merged cloud will be filtered. _Type: bool. Default: false._
- **filters_config**: name of the filter parameters file. _Type: string_


Example: ```point_cloud_processor/config/example_sensor_streams.yaml```


### PointMatcher filter parameters

The filters files are processed by [*libpointmatcher*](https://github.com/anybotics/libpointmatcher). Extensive documentation can be found [here](https://libpointmatcher.readthedocs.io/en/latest/Datafilters/).

Example: ```point_cloud_processor/config/pointmatcher_filters_example.yaml```

## Self Filtering
Filtering of robot links from the URDF can be optionally applied to a `source_stream` after the pointmatcher filters are applied to the point cloud data. Self Filtering relies on links defined in a URDF, and
uses the collision bodies to filter out points within these links. If a link from the URDF is set to be filtered, all points within the collision model of the specified link will be removed from the point
cloud data.

### Self Filtering configuration
The self filtering configuration file specifies a list of links from a URDF to filter from the point cloud data. The format is as follows:
```
timout_usec: <timeout_in_microseconds>
self_see_links:
 - name: <link_name>
   padding: <padding_in_m>
   scale: <scale_ratio>
 - name: <another_link_name>
 ...
```

The configuration must specify at least one named link to filter, but padding and scale values are optional. Padding adds an extra margin around the collision bodies for the link specified in units of meters,
and scaling scales the size of the collision bodies without changing their relative position. The `timeout_usec` parameter specifies how long the self filtering will busy wait for the transform of the first link
to become available in units of microseconds in case publishing of some TFs are delayed. If this timeout is exceeded, the self filtering will not remove any points, and will display a warning. The `timeout_usec`
parameter is optional, and defaults to 1000 microseconds (1 msec).

An additional `self_filter` tag can be added to the URDF file to specify a simpler, hand-tuned geometry to filter a specified link or set of robot links. This is recommended in cases where multiple robot links stand 
in the way of point cloud sensors, and self-filtering based on each individual link would be efficient.

Example config file: ```point_cloud_processor/config/self_filtering_example.yaml```