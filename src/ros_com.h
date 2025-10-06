#ifndef __ROS_COM_H__
#define __ROS_COM_H__

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>

// ROS connection state
extern bool state_connected;

// Command velocity data (from /cmd_vel subscriber)
extern float cmd_linear_x;
extern float cmd_angular_z;

// Motor enable state (from /ugv/motor_enable subscriber)
extern bool motor_enabled;

// Function declarations
void init_ros_msgs();
bool create_entities();
void destroy_entities();
void ros_loop();

#endif
