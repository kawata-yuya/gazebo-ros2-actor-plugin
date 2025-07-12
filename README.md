# gazebo-ros2-actor-plugin
Gazebo ROS2 plugin to move actors using velocity or path messages 

## About

This package contains a plugin for Gazebo Classic (Gazebo 11) and ROS2.
It enables control of **actors** using ROS2 topic communication (`cmd_vel` or `cmd_path`).

<!-- insert demo video -->

## Environment
- Ubuntu 22.04
- ROS Humble
- Gazebo Classic (Gazebo 11)


## Installation

To install this package, please follow the standard **colcon** installation instructions.

```shell
$ mkdir -p {your_ros2_ws}/src
$ cd {your_ros2_ws}/src
$ git clone https://github.com/kawata-yuya/gazebo-ros2-actor-plugin.git
$ cd {your_ros2_ws}
$ colcon build
$ source install/setup.bash
```

## Run Demo
### 1. Start Gazebo

Type this command in a new terminal.

```shell
$ ros2 launch gazebo_ros2_actor_plugin demo_world.launch.py
```

### 2. Publish `cmd_vel` using keyboard

Type this command in other terminal to launch teleop_twisp

```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

<!-- TODO: add node map -->

## Details
### Actor settings

Please add the following content to the world file for Gazebo.  
To spawn multiple actors, you'll need to change the name and topic names for each.

```xml
<actor name="actor1">
  <pose>0 0 1.2138 0 0 0</pose>
  <skin>
    <filename>moonwalk.dae</filename>
    <scale>1.0</scale>
  </skin>
  <animation name="walking">
    <filename>walk.dae</filename>
    <scale>1.000000</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <plugin name="actor_plugin" filename="libgazebo_ros2_actor_command.so">
    <follow_mode>velocity</follow_mode>
    <vel_topic>/cmd_vel</vel_topic>
    <!-- Remap cmd_vel like this -->
    <!-- <vel_topic>/actor1/cmd_vel</vel_topic> --> 
    <path_topic>/cmd_path</path_topic>
    <animation_factor>4.0</animation_factor>
    <linear_tolerance>0.1</linear_tolerance>
    <linear_velocity>1</linear_velocity>
    <angular_tolerance>0.0872</angular_tolerance>
    <angular_velocity>2.5</angular_velocity>
    <default_rotation>1.57</default_rotation>
  </plugin>
</actor>
```

<!-- 
This is a quotation from https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin .
-->
This plugin have the following parameters:

- `follow_mode`: The mode in which the actor will follow the commands. It can be set to either `path` or `velocity`.
- `vel_topic`: The name of the topic to which velocity commands will be published. The default topic name is `/cmd_vel`.
- `path_topic`: The name of the topic to which path commands will be published. The default topic name is `/cmd_path`.
- `animation_factor`: Multiplier to base animation speed that adjusts the speed of both the actor's animation and foot swinging.
- `linear_tolerance`: Maximum allowed distance between actor and target pose during path-following.
- `linear_velocity`: Speed at which actor moves along path during path-following.
- `angular_tolerance`: Maximum allowable difference in orientation between actor's current and desired orientation during rotational alignment.
- `angular_velocity`: Speed at which actor rotates to achieve desired orientation during rotational alignment.
- `default_rotation`: Angle offset for skin collada files. It's set to 1.57 by default but should be adjusted for the skin. It can be changed by adding or subtracting pi/2 to make the actor stand upright. For "DoctorFemaleWalk" actor, the value is "0".


### ROS2 Subscribing Topics

| Topic name | Message type | Work |
|:---|:---|:---|
| `cmd_vel`  | [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) | Give the actor a velocity input |
| `cmd_path` | [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html) | Give the actor a path input |


## Acknowledge
In developing this software, I used the code of the following projects as references.
Thank you to its developers for their excellent work.

- [gazebo-ros-actor-plugin](https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin) by blackcoffeerobotics
