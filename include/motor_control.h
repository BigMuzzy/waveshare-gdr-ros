#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

/**
 * @file motor_control.h
 * @brief Motor control interface for differential drive robot
 *
 * Provides H-bridge PWM motor control using ESP32 LEDC peripheral.
 * Supports bidirectional control with configurable PWM frequency and resolution.
 */

/**
 * @brief Initialize motor control hardware
 *
 * Configures PWM channels and direction control pins for left and right motors.
 * Must be called before any motor control operations.
 *
 * @note PWM frequency is set to 100kHz with 8-bit resolution
 */
void motor_init();

/**
 * @brief Set motor speeds with PWM control
 *
 * @param left_pwm Left motor PWM value (-255 to +255)
 * @param right_pwm Right motor PWM value (-255 to +255)
 *
 * Positive values drive motors forward, negative values reverse.
 * Values below threshold (THRESHOLD_PWM) are treated as zero.
 */
void set_motor_speed(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Emergency stop - immediately halt both motors
 *
 * Sets both direction pins HIGH (brake mode) and PWM to zero.
 * Safe to call at any time, even before motor_init().
 */
void emergency_stop();

#endif // MOTOR_CONTROL_H
