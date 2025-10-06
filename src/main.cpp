#include <Arduino.h>
#include "ros_com.h"

void setup() {
    // Initialize Serial for micro-ROS (CP2102 USB-UART bridge)
    Serial.begin(115200);
    delay(2000);

    // Initialize micro-ROS transport over Serial
    set_microros_serial_transports(Serial);
    delay(2000);

    // Initialize ROS messages
    init_ros_msgs();

    // TODO: Initialize hardware components (motors, encoders, IMU, etc.)
}

void loop() {
    // Handle ROS communication (connection state machine + executor spin)
    ros_loop();

    // TODO: Control loop (motor control, sensor reading, etc.)
}
