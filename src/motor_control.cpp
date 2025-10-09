#include "motor_control.h"
#include "debug_serial.h"

/**
 * Initialize motor control pins and PWM channels
 * Based on reference/general_driver/movtion_module.h::movtionPinInit()
 */
void motorInit() {
    // Configure direction control pins as outputs
    pinMode(BSP::Motor::Left::DIR1_PIN, OUTPUT);
    pinMode(BSP::Motor::Left::DIR2_PIN, OUTPUT);
    pinMode(BSP::Motor::Left::PWM_PIN, OUTPUT);
    pinMode(BSP::Motor::Right::DIR1_PIN, OUTPUT);
    pinMode(BSP::Motor::Right::DIR2_PIN, OUTPUT);
    pinMode(BSP::Motor::Right::PWM_PIN, OUTPUT);

    // Setup PWM channels
    ledcSetup(BSP::Motor::PWM_CHANNEL_A, Motor::PWM_FREQUENCY, Motor::PWM_RESOLUTION_BITS);
    ledcAttachPin(BSP::Motor::Left::PWM_PIN, BSP::Motor::PWM_CHANNEL_A);

    ledcSetup(BSP::Motor::PWM_CHANNEL_B, Motor::PWM_FREQUENCY, Motor::PWM_RESOLUTION_BITS);
    ledcAttachPin(BSP::Motor::Right::PWM_PIN, BSP::Motor::PWM_CHANNEL_B);

    // Initialize all direction pins to LOW (motors stopped)
    digitalWrite(BSP::Motor::Left::DIR1_PIN, LOW);
    digitalWrite(BSP::Motor::Left::DIR2_PIN, LOW);
    digitalWrite(BSP::Motor::Right::DIR1_PIN, LOW);
    digitalWrite(BSP::Motor::Right::DIR2_PIN, LOW);

    debugLog("MOTOR", "PWM channels configured");
    debugPrintf("[MOTOR] Ch A: GPIO%d (%dkHz), Ch B: GPIO%d (%dkHz)\n",
                BSP::Motor::Left::PWM_PIN, Motor::PWM_FREQUENCY / Motor::KHZ_DIVISOR,
                BSP::Motor::Right::PWM_PIN, Motor::PWM_FREQUENCY / Motor::KHZ_DIVISOR);
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
    if (abs(left_pwm) < Motor::THRESHOLD_PWM) {
        // Stop left motor
        digitalWrite(BSP::Motor::Left::DIR1_PIN, LOW);
        digitalWrite(BSP::Motor::Left::DIR2_PIN, LOW);
        ledcWrite(BSP::Motor::PWM_CHANNEL_A, Motor::PWM_ZERO);
    } else if (left_pwm > Motor::PWM_ZERO) {
        // Forward: DIR1=LOW, DIR2=HIGH
        digitalWrite(BSP::Motor::Left::DIR1_PIN, LOW);
        digitalWrite(BSP::Motor::Left::DIR2_PIN, HIGH);
        ledcWrite(BSP::Motor::PWM_CHANNEL_A, abs(left_pwm));
    } else {
        // Reverse: DIR1=HIGH, DIR2=LOW
        digitalWrite(BSP::Motor::Left::DIR1_PIN, HIGH);
        digitalWrite(BSP::Motor::Left::DIR2_PIN, LOW);
        ledcWrite(BSP::Motor::PWM_CHANNEL_A, abs(left_pwm));
    }

    // --- Right Motor (Motor B) ---
    if (abs(right_pwm) < Motor::THRESHOLD_PWM) {
        // Stop right motor
        digitalWrite(BSP::Motor::Right::DIR1_PIN, LOW);
        digitalWrite(BSP::Motor::Right::DIR2_PIN, LOW);
        ledcWrite(BSP::Motor::PWM_CHANNEL_B, Motor::PWM_ZERO);
    } else if (right_pwm > Motor::PWM_ZERO) {
        // Forward: DIR1=LOW, DIR2=HIGH
        digitalWrite(BSP::Motor::Right::DIR1_PIN, LOW);
        digitalWrite(BSP::Motor::Right::DIR2_PIN, HIGH);
        ledcWrite(BSP::Motor::PWM_CHANNEL_B, abs(right_pwm));
    } else {
        // Reverse: DIR1=HIGH, DIR2=LOW
        digitalWrite(BSP::Motor::Right::DIR1_PIN, HIGH);
        digitalWrite(BSP::Motor::Right::DIR2_PIN, LOW);
        ledcWrite(BSP::Motor::PWM_CHANNEL_B, abs(right_pwm));
    }
}

/**
 * Emergency stop - immediately halt both motors
 * Based on reference/general_driver/movtion_module.h::switchEmergencyStop()
 */
void emergencyStop() {
    digitalWrite(BSP::Motor::Left::DIR1_PIN, HIGH);
    digitalWrite(BSP::Motor::Left::DIR2_PIN, HIGH);
    digitalWrite(BSP::Motor::Right::DIR1_PIN, HIGH);
    digitalWrite(BSP::Motor::Right::DIR2_PIN, HIGH);

    ledcWrite(BSP::Motor::PWM_CHANNEL_A, Motor::PWM_ZERO);
    ledcWrite(BSP::Motor::PWM_CHANNEL_B, Motor::PWM_ZERO);
}
