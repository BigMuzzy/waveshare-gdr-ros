#include "motor_control.h"

/**
 * Initialize motor control pins and PWM channels
 * Based on reference/general_driver/movtion_module.h::movtionPinInit()
 */
void motorInit() {
    // Configure direction control pins as outputs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // Setup PWM channels
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWMA, PWM_CHANNEL_A);

    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWMB, PWM_CHANNEL_B);

    // Initialize all direction pins to LOW (motors stopped)
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
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
    left_pwm = constrain(left_pwm, -255, 255);
    right_pwm = constrain(right_pwm, -255, 255);

    // --- Left Motor (Motor A) ---
    if (abs(left_pwm) < 1) {
        // Stop left motor
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        ledcWrite(PWM_CHANNEL_A, 0);
    } else if (left_pwm > 0) {
        // Forward: AIN1=LOW, AIN2=HIGH
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        ledcWrite(PWM_CHANNEL_A, abs(left_pwm));
    } else {
        // Reverse: AIN1=HIGH, AIN2=LOW
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        ledcWrite(PWM_CHANNEL_A, abs(left_pwm));
    }

    // --- Right Motor (Motor B) ---
    if (abs(right_pwm) < 1) {
        // Stop right motor
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        ledcWrite(PWM_CHANNEL_B, 0);
    } else if (right_pwm > 0) {
        // Forward: BIN1=LOW, BIN2=HIGH
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        ledcWrite(PWM_CHANNEL_B, abs(right_pwm));
    } else {
        // Reverse: BIN1=HIGH, BIN2=LOW
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        ledcWrite(PWM_CHANNEL_B, abs(right_pwm));
    }
}

/**
 * Emergency stop - immediately halt both motors
 * Based on reference/general_driver/movtion_module.h::switchEmergencyStop()
 */
void emergencyStop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
}
