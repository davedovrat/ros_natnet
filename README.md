# ROS_NatNet

[OptiTrack](https://optitrack.com/) [NatNet](https://optitrack.com/software/natnet-sdk/) Client that broadcasts [TFs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html).

## Build

This package requires the [NatNet SDK](https://optitrack.com/software/natnet-sdk/) to build and run.
In the following example, the sdk path is /NatNetSDK.

```console
colcon build --cmake-args -DNatNet_SDK_PATH="/NatNetSDK/"
```

## Usage

In the following example, we use a parameter file located in the working directory.

```console
ros2 run ros_natnet broadcast --ros-args --params-file ./ros_natnet_broadcaster.yaml
```

The executable links with the [NatNet SDK](https://optitrack.com/software/natnet-sdk/) library (libNatNet.so), so make sure the library can be found at runtime (e.g. it is in /usr/local/lib or anywhere in one of the LD_LIBRARY_PATH environment variable paths).

## Parameters

The parameter file should be in ROS-parameter-file format, like this:

```yaml
/ros_natnet_broadcaster:
  ros__parameters:
    location:
    - 0.0
    - 0.0
    - 0.0
    orientation:
    - 1.0
    - 0.0
    - 0.0
    - 1.0
    parent_frame: world
    root_frame: mocap
    static_tf_hz: 1
    use_sim_time: false
```

| Parameter 	| Meaning		|Default Value	|	Remark	|
| ---------		| ------------	|-------		|-------	|
| parent_frame	| The parent frame of the root frame							| odom	| for example, when working with a single turtlebot, this sets the turtlebot's initial position relative to the motion capture systems' (mocap's) frame.	|
| root_frame	| Everything the motion capture systems (mocap) measures will be relative to this frame	| mocap	|
| location	| The root frame's location in the parent frame's 3D space coordinates	| [0, 0, 0]	| 3D vector (x,y,z)	|
| orientation	| The root frame's orientation represented by a quaternion	| [0.5, 0.5, 0.5, 0.5]	| Quaternion (x,y,z,w), this is useful if your application co-exists with others using different 'up' conventions.	|
| static_tf_hz	| The frequency for broadcasting the static_tf between the root and parent frames | 1 | in hertz (Hz)|
| use_sim_time	| Should the time also be simulated	| false	|	See [ROS Clock](http://wiki.ros.org/Clock) |

## Dependencies

| Dependency                                                            | Minimal Version   | License |
| ------------                                                          | ---------------   | ------- |
| [NatNet SDK](https://optitrack.com/software/natnet-sdk/)               | 4.0               | [OptiTrack EULA](https://optitrack.com/about/legal/eula.html) |

### ROS Dependencies

* rclcpp
* geometry_msgs
* std_msgs
* geometry_msgs
* tf2
* tf2_ros
