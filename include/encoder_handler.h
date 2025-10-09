#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <ESP32Encoder.h>
#include "config.h"

// All encoder constants are now in config.h
// Access via Encoder::LEFT_A_PIN, Encoder::PULSES_PER_REV, etc.
// Robot physical parameters: Robot::WHEEL_DIAMETER_M, Robot::WHEEL_BASE_M, etc.

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
