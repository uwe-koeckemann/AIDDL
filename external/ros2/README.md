# AIDDL External ROS2 Library

This library provides conversions between the AI Domain Definition Language (AIDDL) framework and ROS2.
The main purpose of these converters is to be used in the `aiddl_ros2_bridge` ROS2 package (TODO: link)
and ultimately decouple the AIDDL framework from ROS throught `aiddl_external_grpc`.
ROS2 nodes from the `aiddl_ros2_bridge` can be configured to dynamically load converters. 
The overall envisioned usage of these packages is:

1. Install `aiddl_external_ros2`
2. Clone & build `aiddl_ros2_bridge`
3. *Optional:* write custom converters if needed (for custom ROS2 messages, services, or actions).
  - Write a simple Python package with one class per message, service, or action and install it locally
  - Use `aiddl_external_ros2` to convert existing messages where possible (e.g., `Pose`, `PoseStamped`)
4. Write a YAML file describing ROS2 nodes and their AIDDL converters (from this package)
5. Launch all ROS2 nodes and control your robots through `aiddl_external_grpc`


## Overview 

Interaction with ROS2 comes in three flavors: messages, services and actions. For each of these we use 
different **static** methods.

### Messages

Messages can be converted between AIDDL and ROS2 formats. 
The two methods for this are `ros2aiddl` and `aiddl2ros`.
Thw following example shows how strings are converted.

    class StringConverter:
        @staticmethod
        def ros2aiddl(string_msg):
            return Str(string_msg)

        @staticmethod
        def aiddl2ros(string):
            return string.string

### Services

Services allow to call functions in ROS2 so we need to convert the arguments of the service call from AIDDL
to ROS and the result from ROS to AIDDL.
The two methods for this are `request_aiddl2ros` and `result_ros2aiddl` and their implementation is 
exactly as for messages.
Usually service converters rely heavily on message converters (since their inputs and outputs are ROS messages).

### Actions

ROS2 actions allow sending **goals** to action servers that are then executed over a period time. 
During execution, **feedback** messages can be provided and after execution a **result** message is sent.
So overall we need three converters which are `request_aiddl2ros` to accept goals, `feedback_ros2aiddl` to 
convert feedback, and `result_aiddl2ros` to convert the result.

The following example converts from AIDDL to the Nav2 action `NavigateToPose`. (The result message of the action is empty, so 
the corresponding converter simply returns the constant symbol `NIL`.)


    class NavigateToPoseConverter(object):
        @staticmethod
        def request_aiddl2ros(term: Term):
            msg = NavigateToPose.Goal()
            msg.pose = PoseStampedConverter.aiddl2ros(term[0])
            msg.behavior_tree = term[1].unpack()
            return msg
    
        @staticmethod
        def feedback_ros2aiddl(msg):
            msg = NavigateToPose.Feedback()
            current_pose_term = PoseStampedConverter.ros2aiddl(msg.current_pose)
            navigation_time = DurationConverter.ros2aiddl(msg.navigation_time)
            estimated_time_remaining = DurationConverter.ros2aiddl(msg.estimated_time_remaining)
            number_of_recoveries = Int(msg.number_of_recoveries)
            distance_remaining = Real(msg.distance_remaining)
            return List(
                KeyVal(Sym("current_pose"), current_pose_term),
                KeyVal(Sym("navigation_time"), navigation_time),
                KeyVal(Sym("estimated_time_remaining"), estimated_time_remaining),
                KeyVal(Sym("number_of_recoveries"), number_of_recoveries),
                KeyVal(Sym("distance_remaining"), distance_remaining)
            )
    
        @staticmethod
        def result_ros2aiddl(msg):
            return Sym("NIL")

## Available Converters

The following table lists all available converters with the full module and class name so they can be directly
used in the configuration of `aiddl_ros2_bridge` nodes.

### Messages

Messages can be translated from AIDDL to ROS (A->R) and vice versa (R->A). The *Direction* column indicates which of these options 
are implemented in the current version. In most cases *both* should be supported, but some ROS topics (e.g., visualization
markers) may be supported for publishing from AIDDL to ROS only since we do not anticipate wanting to read them ourselves.

| ROS2 Class                      | Converter Class                                        | Direction |
|---------------------------------|--------------------------------------------------------|-----------|
| builtin_interfaces.msg.Duration | aiddl_external_ros2.msg.builtin.DurationConverter      | both      | 
| builtin_interfaces.msg.Time     | aiddl_external_ros2.msg.builtin.TimeConverter          | both      | 
| std_msgs.msg.Header             | aiddl_external_ros2.msg.std.HeaderConverter            | both      |
| std_msgs.msg.String             | aiddl_external_ros2.msg.std.StringConverter            | both      |
| geometry_msgs.msg.Point         | aiddl_external_ros2.msg.geometry.PointConverter        | both      |
| geometry_msgs.msg.Pose          | aiddl_external_ros2.msg.geometry.PoseConverter         | both      |
| geometry_msgs.msg.PoseStamped   | aiddl_external_ros2.msg.geometry.PoseStamped Converter | both      |
| nav_msgs.msg.MapMetaData        | aiddl_external_ros2.msg.nav.MapMetaDataConverter       | R->A      |
| nav_msgs.msg.OccupancyGrid      | aiddl_external_ros2.msg.nav.OccupancyGridConverter     | R->A      |
| visualization_msgs.msg.Marker   | aiddl_external_ros2.msg.marker.MarkerConverter         | A->R      |

### Services

| ROS2 Class          | Converter Class                                 |
|---------------------|-------------------------------------------------|
| nav_msgs.srv.GetMap | aiddl_external_ros2.service.nav.GetMapConverter |

### Actions

| ROS2 Class                        | Converter Class                                         |
|-----------------------------------|---------------------------------------------------------|
| nav2_msgs.action.NavigateToPose   | aiddl_external_ros2.action.nav2.NavigateToPoseConverter |

## Open Issues

- Support direct conversions from ROS to B to avoid intermediate AIDDL conversion
- Add remaining converters from `std_msgs`

## Version History

### 0.2.1

- Move constants to appropriate sub-packages
- Fix package structure to reflect ROS2 structure
- Documentation

### 0.1.0 

- Ported functionality from ROS1 version
