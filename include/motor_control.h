#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

// Use constants from config.h instead of #define macros
// All motor pin and PWM configurations are now in config.h
// Access via Motor::PWMA_PIN, Motor::PWM_FREQUENCY, etc.

// Motor control functions
void motorInit();
void setMotorSpeed(int16_t left_pwm, int16_t right_pwm);
void emergencyStop();

#endif // MOTOR_CONTROL_H
