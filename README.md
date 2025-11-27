# Ryder Robots Joystick Node

## Overview
The Ryder Robots Joystick Node is a ROS2 lifecycle node that provides communication bridges to interface with `sensor_msgs::msg::Joy` messages. These messages are published on the topic `/[namespace]/joy` and enable manual control of robots, primarily useful during the development stage.

## Features
- Publishes joystick input as `sensor_msgs::msg::Joy` messages.
- Supports namespaced topics for flexible multi-robot control.
- Designed for manual robot control in ROS2 environments.
- Suitable for integration with various robot sensor and control systems.

## Usage
Run the node within your ROS2 workspace, making sure the joystick device is correctly set up and accessible. The node will listen to the joystick device and publish messages to the `/[namespace]/joy` topic.

Example command to run the node (replace `[namespace]` with your robot's namespace):


```bash
ros2 run rr_joystick rr_joystick_pub --ros-args --log-level DEBUG   -p transport_plugin:=rr_joy_udp_plugin 
```

```bash
ros2 lifecycle set /rr_joystick_node configure
ros2 lifecycle set /rr_joystick_node activate
```

Stopping node can be achieved with the following:

```bash
ros2 lifecycle set /rr_joystick_node deactivate
ros2 lifecycle set /rr_joystick_node cleanup
ros2 lifecycle set /rr_joystick_node shutdown
```

## Development

Command for compiling in debug mode:

```bash
colcon build --packages-select rr_joystick --cmake-args -DCMAKE_BUILD_TYPE=Debug 
ros2 run --prefix 'gdbserver localhost:3000' rr_joystick rr_joystick_pub --ros-args --log-level debug  -p transport_plugin:=rr_joy_udp_plugin

## Transport Plugin

Transport plugin(s) allow for abstraction of plumbing from the node to the hardware
transport layer, note that for Joysticks only UDP transport plugin is available.
