#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <ESP32Encoder.h>

// Pin definitions (from reference/general_driver/ugv_config.h)
#define AENCA 35             // Left encoder A
#define AENCB 34             // Left encoder B
#define BENCA 27             // Right encoder A
#define BENCB 16             // Right encoder B

// Robot parameters
// NOTE: These values are from the General Driver default configuration
// Adjust if your robot has different specifications!
#define WHEEL_DIAMETER 0.065      // meters (65mm) - measure your actual wheel
#define PULSES_PER_REV 180        // encoder pulses per wheel revolution
#define WHEEL_BASE 0.200          // meters (distance between left/right wheels)

// Encoder initialization
void encoderInit();

// Get raw encoder counts
void getEncoderCounts(int32_t &left, int32_t &right);

// Get wheel speeds in m/s
void getWheelSpeeds(float &left_speed, float &right_speed);

// Reset encoder counts to zero
void resetEncoders();

// Global encoder objects
extern ESP32Encoder encoderLeft;
extern ESP32Encoder encoderRight;

#endif // ENCODER_HANDLER_H
