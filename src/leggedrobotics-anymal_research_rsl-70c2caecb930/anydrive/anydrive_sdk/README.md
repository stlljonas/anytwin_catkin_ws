# ANYdrive SDK

*ANYdrive* Software Development Kit.

The source code is released under a [proprietary license](LICENSE).

The software has been developed and tested with [Ubuntu 16.04 LTS](http://www.releases.ubuntu.com/16.04/) and [Ubuntu 18.04 LTS](http://www.releases.ubuntu.com/18.04/).
The ROS-dependent packages require [ROS Kinetic](http://wiki.ros.org/kinetic) or [ROS Melodic](http://wiki.ros.org/melodic).

**Authors:** Remo Diethelm, Christian Gehring, Samuel Bachmann

## Documentation

Please read the [ANYdrive User Manual](https://anybotics-anydrive-doc.readthedocs-hosted.com/en/latest/index.html) before using the device.

Additional information can be found in the [API documentation](https://docs.leggedrobotics.com/anydrive_sdk_doc/).

## Packages

The following sections briefly describe the content of each package.

### anydrive

Core C++ library to interface one or multiple drives.
Contains abstract base classes for the communication interface.

### anydrive_ethercat

C++ library implementing the abstract communication interface classes in the *anydrive* library for EtherCAT.

### anydrive_msgs

ROS message and service definitions.

### anydrive_ros

C++ library adding a ROS interface to the classes in the *anydrive* library.

### anydrive_ethercat_ros

C++ library combining *anydrive_ethercat* and *anydrive_ros*.
Contains a ROS node.

### anydrive_monitor

Graphical user interface library to supervise and command one or multiple drives.

### rqt_anydrive_monitor

C++ rqt library adding a ROS interface to the classes in the *anydrive_monitor* library.

### anydrive_sdk_doc

Doxygen documentation of the ANYdrive SDK.


## Installation

Install the latest release with
```
sudo apt install ros-$ROS_DISTRO-anydrive-ethercat-ros ros-$ROS_DISTRO-rqt-anydrive-monitor
```

To build from source, download the newest version from [Bitbucket](https://bitbucket.org/leggedrobotics/anydrive_sdk),
link the packages into your catkin workspace and run
```
catkin build anydrive_ethercat_ros rqt_anydrive_monitor
```

## Setup

### Allow your user to set priorities and niceness

The provided executables perform best when they are given higher priority than other processes.
By default, a linux user or group cannot set priorities higher than 0.
To allow your linux user account to do so, you need to append the following entries to the ``/etc/security/limits.conf`` file (replacing ``<username>``):

```
<username>       -       rtprio          99
<username>       -       nice            -20
```

To allow your entire group to set higher priorities, append (replacing ``<groupname>``):

```
@<groupname>     -       rtprio          99
@<groupname>     -       nice            -20
```

## Usage

The typical usage is to run the *anydrive_ethercat_ros* ROS node.
It requires the following files:

- A single *ANYdrive* setup file
- One or multiple *ANYdrive* configuration file(s)

Examples can be found in ``anydrive/example_setups``.

### The ANYdrive setup file

The *ANYdrive* setup file contains information about the number of drives, the interfaces where they are connected as well as their configuration files.
This file has to be called *setup.yaml*.

### The ANYdrive configuration files

Each *ANYdrive* within the *ANYdrive* setup needs a configuration file.
It is possible to use the same configuration file for multiple drives.

### Running the ROS node

The *anydrive_ethercat_ros* ROS node can be started with
```
roslaunch anydrive_ethercat_ros anydrive_ethercat_ros.launch setup_file:=/path/to/your/setup/file.yaml
```

In the case of EtherCAT, the application requires sudo rights to open the raw socket.
With the following command, the executable can be given sudo rights:
```
sudo setcap cap_net_raw+ep /path/to/anydrive_ethercat_ros_node
```

Since these rights expire every time the executable is overwritten, there is an alternative launch file to automate the procedure:
```
roslaunch anydrive_ethercat_ros anydrive_ethercat_ros_setcap.launch setup_file:=/path/to/setup/file.yaml password:="abc123"
```

*Note:* Your command line terminal might save the command history containing your password.

### Running the rqt GUI

To supervise and control the *ANYdrives* within the ROS node, the rqt GUI can be launched with
```
roslaunch rqt_anydrive_monitor rqt_anydrive_monitor.launch setup_file:=/path/to/your/setup/file.yaml
```

### Running the statusword parser

To convert statuswords into human readable text, type
```
rosrun anydrive parse_statusword 727
```
