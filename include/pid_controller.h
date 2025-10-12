#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>  // br3ttb/PID library
#include "config.h"

/**
 * @file pid_controller.h
 * @brief PID speed control for differential drive motors
 *
 * Implements feedforward + PID control for accurate speed tracking.
 * Uses br3ttb/PID library for PID computation.
 */

/**
 * @brief Initialize PID controllers for both wheels
 *
 * Sets output limits, enables automatic mode, and configures
 * PID gains from PIDConfig namespace.
 */
void pid_init();

/**
 * @brief Compute PID outputs for both wheels
 *
 * @param left_setpoint Target speed for left wheel (m/s)
 * @param right_setpoint Target speed for right wheel (m/s)
 * @param left_speed Measured speed of left wheel (m/s)
 * @param right_speed Measured speed of right wheel (m/s)
 * @param left_output Reference to store left motor PWM output
 * @param right_output Reference to store right motor PWM output
 *
 * Uses feedforward + PID correction for improved tracking performance.
 */
void pid_compute(float left_setpoint, float right_setpoint,
                 float left_speed, float right_speed,
                 int16_t &left_output, int16_t &right_output);

/**
 * @brief Reset PID controllers
 *
 * Clears integral windup and resets internal state.
 * Useful after large setpoint changes or emergency stops.
 */
void pid_reset();

/**
 * @brief Update PID tuning parameters
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 *
 * Dynamically adjusts PID gains for both wheels.
 */
void set_pid_tunings(float kp, float ki, float kd);

// Global PID controller objects (using PID v1 library)
// Note: Keep camelCase names to match external library convention
extern PID pidLeft;
extern PID pidRight;

#endif // PID_CONTROLLER_H
