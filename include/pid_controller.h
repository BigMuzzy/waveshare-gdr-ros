#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>  // br3ttb/PID library

// PID gains (from reference/general_driver/ugv_config.h)
// These values are tuned for the General Driver Board
#define PID_KP 20.0
#define PID_KI 2000.0
#define PID_KD 0.0

// Output limits (PWM range)
#define PID_OUTPUT_MIN -255
#define PID_OUTPUT_MAX 255

// Minimum PWM threshold (below this, motor won't move effectively)
#define THRESHOLD_PWM 23

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
