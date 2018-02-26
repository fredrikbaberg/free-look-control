# Teleoperation package for Free Look Control

## Setup

Parameters are read from ROS parameter server on launch, available parameters are listed below, with default value:

- Publishing to:
- - ~joy_topic (default: 'flc_joy')
- - ~odom_topic (default: 'odom')
- - ~virtual_camera_topic (default: 'viz/virtual_camera')
- - ~output_base (default: 'cmd_vel')
- - ~camera_pan_topic (default: 'camera_pan')
- - ~camera_tilt_topic (default: 'camera_tilt')
- Parameters:
- - ~ugv_L (default: 0.15)
- - ~ugv_d (default: 0.42)
- - ~vel_max (default: 0.3)
- - ~ang_max (default: 0.3)
- - ~pan_offset (default: -54.0)
- - ~tilt_offset (default: 90.0)

## Launch

This can either be run as a normal node, or using ActionLib. In case of ActionLib, it is necessary to have a client call the server for FLC to start and publish data.

## Using

Intended to be used with an Xbox 360 Wired controller (currently only tested with Xbox One controller). LB is used as deadmans button.

- Deadmans button needs to be pressed in order to move the robot and/or camera.
- Left joystick translate the platform (relative to camera orientation),
- Right joystick controls camera pan and tilt.

