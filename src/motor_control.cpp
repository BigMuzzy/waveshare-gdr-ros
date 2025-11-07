/**
 * @file motor_control.cpp
 * @brief Motor PWM control and direction management implementation
 *
 * Implements H-bridge motor control using ESP32 LEDC peripheral.
 * Based on reference/general_driver/movtion_module.h
 */

#include "motor_control.h"
#include "debug_serial.h"

void motor_init() {
    // Configure direction control pins as outputs
    pinMode(BSP::MotorDriver::Left::DIR1_PIN, OUTPUT);
    pinMode(BSP::MotorDriver::Left::DIR2_PIN, OUTPUT);
    pinMode(BSP::MotorDriver::Left::PWM_PIN, OUTPUT);
    pinMode(BSP::MotorDriver::Right::DIR1_PIN, OUTPUT);
    pinMode(BSP::MotorDriver::Right::DIR2_PIN, OUTPUT);
    pinMode(BSP::MotorDriver::Right::PWM_PIN, OUTPUT);

    // Setup PWM channels
    ledcSetup(BSP::MotorDriver::PWM_CHANNEL_A, Motor::PWM_FREQUENCY, Motor::PWM_RESOLUTION_BITS);
    ledcAttachPin(BSP::MotorDriver::Left::PWM_PIN, BSP::MotorDriver::PWM_CHANNEL_A);

    ledcSetup(BSP::MotorDriver::PWM_CHANNEL_B, Motor::PWM_FREQUENCY, Motor::PWM_RESOLUTION_BITS);
    ledcAttachPin(BSP::MotorDriver::Right::PWM_PIN, BSP::MotorDriver::PWM_CHANNEL_B);

    // Initialize all direction pins to LOW (motors stopped)
    digitalWrite(BSP::MotorDriver::Left::DIR1_PIN, LOW);
    digitalWrite(BSP::MotorDriver::Left::DIR2_PIN, LOW);
    digitalWrite(BSP::MotorDriver::Right::DIR1_PIN, LOW);
    digitalWrite(BSP::MotorDriver::Right::DIR2_PIN, LOW);

    DEBUG_LOG_INIT("[MOTOR]", "PWM channels configured");
    debug_printf("[MOTOR] Ch A: GPIO%d (%dkHz), Ch B: GPIO%d (%dkHz)\n",
                 BSP::MotorDriver::Left::PWM_PIN, Motor::PWM_FREQUENCY / Motor::KHZ_DIVISOR,
                 BSP::MotorDriver::Right::PWM_PIN, Motor::PWM_FREQUENCY / Motor::KHZ_DIVISOR);
}

void set_motor_speed(int16_t left_pwm, int16_t right_pwm) {
    // Clamp PWM values to safe range
    left_pwm = constrain(left_pwm, Motor::PWM_MIN, Motor::PWM_MAX);
    right_pwm = constrain(right_pwm, Motor::PWM_MIN, Motor::PWM_MAX);

    // --- Left Motor (Motor A) ---
    if (abs(left_pwm) < Motor::THRESHOLD_PWM) {
        // Stop left motor
        digitalWrite(BSP::MotorDriver::Left::DIR1_PIN, LOW);
        digitalWrite(BSP::MotorDriver::Left::DIR2_PIN, LOW);
        ledcWrite(BSP::MotorDriver::PWM_CHANNEL_A, Motor::PWM_ZERO);
    } else {
        if (left_pwm < Motor::PWM_ZERO) {
            // Forward: DIR1=LOW, DIR2=HIGH
            digitalWrite(BSP::MotorDriver::Left::DIR1_PIN, LOW);
            digitalWrite(BSP::MotorDriver::Left::DIR2_PIN, HIGH);
        } else {
            // Reverse: DIR1=HIGH, DIR2=LOW
            digitalWrite(BSP::MotorDriver::Left::DIR1_PIN, HIGH);
            digitalWrite(BSP::MotorDriver::Left::DIR2_PIN, LOW);
        }
        ledcWrite(BSP::MotorDriver::PWM_CHANNEL_A, abs(left_pwm));
    }
    // --- Right Motor (Motor B) ---
    if (abs(right_pwm) < Motor::THRESHOLD_PWM) {
        // Stop right motor
        digitalWrite(BSP::MotorDriver::Right::DIR1_PIN, LOW);
        digitalWrite(BSP::MotorDriver::Right::DIR2_PIN, LOW);
        ledcWrite(BSP::MotorDriver::PWM_CHANNEL_B, Motor::PWM_ZERO);
    } else {
        if (right_pwm < Motor::PWM_ZERO) {
            // Forward: DIR1=LOW, DIR2=HIGH
            digitalWrite(BSP::MotorDriver::Right::DIR1_PIN, LOW);
            digitalWrite(BSP::MotorDriver::Right::DIR2_PIN, HIGH);
        } else {
            // Reverse: DIR1=HIGH, DIR2=LOW
            digitalWrite(BSP::MotorDriver::Right::DIR1_PIN, HIGH);
            digitalWrite(BSP::MotorDriver::Right::DIR2_PIN, LOW);
        }
        ledcWrite(BSP::MotorDriver::PWM_CHANNEL_B, abs(right_pwm));
    }
}

void emergency_stop() {
    digitalWrite(BSP::MotorDriver::Left::DIR1_PIN, HIGH);
    digitalWrite(BSP::MotorDriver::Left::DIR2_PIN, HIGH);
    digitalWrite(BSP::MotorDriver::Right::DIR1_PIN, HIGH);
    digitalWrite(BSP::MotorDriver::Right::DIR2_PIN, HIGH);

    ledcWrite(BSP::MotorDriver::PWM_CHANNEL_A, Motor::PWM_ZERO);
    ledcWrite(BSP::MotorDriver::PWM_CHANNEL_B, Motor::PWM_ZERO);
}
