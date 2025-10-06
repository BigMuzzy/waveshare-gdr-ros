#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Pin definitions (from reference/general_driver/ugv_config.h)
#define PWMA 25              // Motor A (Left) PWM control
#define AIN1 21              // Motor A input 1
#define AIN2 17              // Motor A input 2
#define PWMB 26              // Motor B (Right) PWM control
#define BIN1 22              // Motor B input 1
#define BIN2 23              // Motor B input 2

// PWM configuration
#define PWM_FREQ 100000      // 100kHz
#define PWM_CHANNEL_A 5
#define PWM_CHANNEL_B 6
#define PWM_RESOLUTION 8     // 8-bit (0-255)

// Motor control functions
void motorInit();
void setMotorSpeed(int16_t left_pwm, int16_t right_pwm);
void emergencyStop();

#endif // MOTOR_CONTROL_H
