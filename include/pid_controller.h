#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>  // br3ttb/PID library
#include "config.h"

// All PID constants are now in config.h
// Access via PIDConfig::KP, PIDConfig::KI, PIDConfig::KD, PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX, PIDConfig::THRESHOLD

// PID controller initialization
void pidInit();

// Compute PID outputs for both wheels
void pidCompute(float left_setpoint, float right_setpoint,
                float left_speed, float right_speed,
                int16_t &left_output, int16_t &right_output);

// Reset PID controllers (clear integral term)
void pidReset();

// Update PID tuning parameters
void setPIDTunings(float kp, float ki, float kd);

// Global PID controller objects (using PID v1 library)
extern PID pidLeft;
extern PID pidRight;

#endif // PID_CONTROLLER_H
