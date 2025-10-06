#include <Arduino.h>
#include "ros_com.h"
#include "motor_control.h"
#include "encoder_handler.h"
#include "pid_controller.h"
#include "kinematics.h"

// Global variables for control loop
float target_left_speed = 0.0;   // Target speed for left wheel (m/s)
float target_right_speed = 0.0;  // Target speed for right wheel (m/s)

void setup() {
    // Initialize Serial for micro-ROS (CP2102 USB-UART bridge)
    Serial.begin(115200);
    delay(2000);

    // Initialize micro-ROS transport over Serial
    set_microros_serial_transports(Serial);
    delay(2000);

    // Initialize ROS messages
    init_ros_msgs();

    // Initialize hardware components
    motorInit();        // H-bridge PWM setup
    encoderInit();      // Encoder interrupt setup
    pidInit();          // PID controller initialization

    // Reset encoders to zero
    resetEncoders();
}

void loop() {
    // Handle ROS communication (connection state machine + executor spin)
    ros_loop();

    // Control loop - only run when ROS agent is connected
    if (state_connected) {
        // 1. Convert cmd_vel to individual wheel speeds using differential drive kinematics
        twistToWheelSpeeds(cmd_linear_x, cmd_angular_z,
                          target_left_speed, target_right_speed);

        // 2. Read actual wheel speeds from encoders
        float actual_left_speed, actual_right_speed;
        getWheelSpeeds(actual_left_speed, actual_right_speed);

        // 3. Compute PID control to calculate PWM outputs
        int16_t left_pwm, right_pwm;
        pidCompute(target_left_speed, target_right_speed,
                   actual_left_speed, actual_right_speed,
                   left_pwm, right_pwm);

        // 4. Apply motor commands (if motors are enabled)
        if (motor_enabled) {
            setMotorSpeed(left_pwm, right_pwm);
        } else {
            emergencyStop();
        }
    } else {
        // When disconnected from ROS agent, stop motors for safety
        emergencyStop();
    }
}
