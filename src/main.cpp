#include <Arduino.h>
#include "ros_com.h"
#include "motor_control.h"
#include "encoder_handler.h"
#include "pid_controller.h"
#include "kinematics.h"
#include "debug_serial.h"

// Global variables for control loop
float target_left_speed = 0.0;   // Target speed for left wheel (m/s)
float target_right_speed = 0.0;  // Target speed for right wheel (m/s)

void setup() {
    // Initialize debug serial FIRST (for early debugging on GPIO5/4)
    debug_serial_init(BSP::UART::Debug::BAUD_RATE);
    debug_log("INIT", "System starting...");

    // Initialize Serial for micro-ROS (CP2102 USB-UART bridge)
    Serial.begin(BSP::UART::BAUD_RATE);
    delay(BSP::UART::INIT_DELAY_MS);

    // Initialize micro-ROS transport over Serial
    debug_log("INIT", "Setting up micro-ROS transport...");
    set_microros_serial_transports(Serial);
    delay(BSP::UART::INIT_DELAY_MS);

    // Initialize ROS messages
    debug_log("INIT", "Initializing ROS messages...");
    init_ros_msgs();

    // Initialize hardware components
    debug_log("INIT", "Initializing motors...");
    motor_init();        // H-bridge PWM setup

    debug_log("INIT", "Initializing encoders...");
    encoder_init();      // Encoder interrupt setup

    debug_log("INIT", "Initializing PID...");
    pid_init();          // PID controller initialization

    // Reset encoders to zero
    reset_encoders();

    debug_log("INIT", "System ready!");
    debug_println();
}

void loop() {
    // Handle ROS communication (connection state machine + executor spin)
    ros_loop();

    // Control loop - only run when ROS agent is connected
    if (state_connected) {
        // 1. Convert cmd_vel to individual wheel speeds using differential drive kinematics
        twist_to_wheel_speeds(cmd_linear_x, cmd_angular_z,
                          target_left_speed, target_right_speed);

        // 2. Read actual wheel speeds from encoders
        float actual_left_speed, actual_right_speed;
        get_wheel_speeds(actual_left_speed, actual_right_speed);

        // 3. Compute PID control to calculate PWM outputs
        int16_t left_pwm, right_pwm;
        pid_compute(target_left_speed, target_right_speed,
                   actual_left_speed, actual_right_speed,
                   left_pwm, right_pwm);

        // 4. Apply motor commands (if motors are enabled)
        if (motor_enabled) {
            set_motor_speed(left_pwm, right_pwm);
        } else {
            emergency_stop();
        }
    } else {
        // When disconnected from ROS agent, stop motors for safety
        emergency_stop();
    }
}
