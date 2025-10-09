#include "motor_control.h"
#include "debug_serial.h"

/**
 * Initialize motor control pins and PWM channels
 * Based on reference/general_driver/movtion_module.h::movtionPinInit()
 */
void motorInit() {
    // Configure direction control pins as outputs
    pinMode(Motor::AIN1_PIN, OUTPUT);
    pinMode(Motor::AIN2_PIN, OUTPUT);
    pinMode(Motor::PWMA_PIN, OUTPUT);
    pinMode(Motor::BIN1_PIN, OUTPUT);
    pinMode(Motor::BIN2_PIN, OUTPUT);
    pinMode(Motor::PWMB_PIN, OUTPUT);

    // Setup PWM channels
    ledcSetup(Motor::PWM_CHANNEL_A, Motor::PWM_FREQUENCY, Motor::PWM_RESOLUTION_BITS);
    ledcAttachPin(Motor::PWMA_PIN, Motor::PWM_CHANNEL_A);

    ledcSetup(Motor::PWM_CHANNEL_B, Motor::PWM_FREQUENCY, Motor::PWM_RESOLUTION_BITS);
    ledcAttachPin(Motor::PWMB_PIN, Motor::PWM_CHANNEL_B);

    // Initialize all direction pins to LOW (motors stopped)
    digitalWrite(Motor::AIN1_PIN, LOW);
    digitalWrite(Motor::AIN2_PIN, LOW);
    digitalWrite(Motor::BIN1_PIN, LOW);
    digitalWrite(Motor::BIN2_PIN, LOW);

    debugLog("MOTOR", "PWM channels configured");
    debugPrintf("[MOTOR] Ch A: GPIO%d (%dkHz), Ch B: GPIO%d (%dkHz)\n",
                Motor::PWMA_PIN, Motor::PWM_FREQUENCY / Motor::KHZ_DIVISOR,
                Motor::PWMB_PIN, Motor::PWM_FREQUENCY / Motor::KHZ_DIVISOR);
}

/**
 * Set motor speeds with direction control
 * @param left_pwm  Left motor PWM value (-255 to +255)
 * @param right_pwm Right motor PWM value (-255 to +255)
 *
 * Positive values = forward
 * Negative values = reverse
 *
 * Based on reference/general_driver/movtion_module.h::leftCtrl() and rightCtrl()
 */
void setMotorSpeed(int16_t left_pwm, int16_t right_pwm) {
    // Clamp PWM values to safe range
    left_pwm = constrain(left_pwm, Motor::PWM_MIN, Motor::PWM_MAX);
    right_pwm = constrain(right_pwm, Motor::PWM_MIN, Motor::PWM_MAX);

    // --- Left Motor (Motor A) ---
    if (abs(left_pwm) < Motor::PWM_DEADZONE) {
        // Stop left motor
        digitalWrite(Motor::AIN1_PIN, LOW);
        digitalWrite(Motor::AIN2_PIN, LOW);
        ledcWrite(Motor::PWM_CHANNEL_A, Motor::PWM_ZERO);
    } else if (left_pwm > Motor::PWM_ZERO) {
        // Forward: AIN1=LOW, AIN2=HIGH
        digitalWrite(Motor::AIN1_PIN, LOW);
        digitalWrite(Motor::AIN2_PIN, HIGH);
        ledcWrite(Motor::PWM_CHANNEL_A, abs(left_pwm));
    } else {
        // Reverse: AIN1=HIGH, AIN2=LOW
        digitalWrite(Motor::AIN1_PIN, HIGH);
        digitalWrite(Motor::AIN2_PIN, LOW);
        ledcWrite(Motor::PWM_CHANNEL_A, abs(left_pwm));
    }

    // --- Right Motor (Motor B) ---
    if (abs(right_pwm) < Motor::PWM_DEADZONE) {
        // Stop right motor
        digitalWrite(Motor::BIN1_PIN, LOW);
        digitalWrite(Motor::BIN2_PIN, LOW);
        ledcWrite(Motor::PWM_CHANNEL_B, Motor::PWM_ZERO);
    } else if (right_pwm > Motor::PWM_ZERO) {
        // Forward: BIN1=LOW, BIN2=HIGH
        digitalWrite(Motor::BIN1_PIN, LOW);
        digitalWrite(Motor::BIN2_PIN, HIGH);
        ledcWrite(Motor::PWM_CHANNEL_B, abs(right_pwm));
    } else {
        // Reverse: BIN1=HIGH, BIN2=LOW
        digitalWrite(Motor::BIN1_PIN, HIGH);
        digitalWrite(Motor::BIN2_PIN, LOW);
        ledcWrite(Motor::PWM_CHANNEL_B, abs(right_pwm));
    }
}

/**
 * Emergency stop - immediately halt both motors
 * Based on reference/general_driver/movtion_module.h::switchEmergencyStop()
 */
void emergencyStop() {
    digitalWrite(Motor::AIN1_PIN, HIGH);
    digitalWrite(Motor::AIN2_PIN, HIGH);
    digitalWrite(Motor::BIN1_PIN, HIGH);
    digitalWrite(Motor::BIN2_PIN, HIGH);

    ledcWrite(Motor::PWM_CHANNEL_A, Motor::PWM_ZERO);
    ledcWrite(Motor::PWM_CHANNEL_B, Motor::PWM_ZERO);
}
