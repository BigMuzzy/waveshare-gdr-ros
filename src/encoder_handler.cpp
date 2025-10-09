#include "encoder_handler.h"
#include "debug_serial.h"
#include <Arduino.h>

// Encoder objects (instantiated here, declared extern in header)
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// Speed calculation variables
static unsigned long lastTime = 0;
static int32_t lastCountLeft = 0;
static int32_t lastCountRight = 0;
static float speedLeft = 0.0;
static float speedRight = 0.0;

// Conversion factor: pulses to meters (from config.h Robot namespace)
static const float PULSES_TO_METERS = Robot::METERS_PER_PULSE;

/**
 * Initialize encoders
 * Based on reference/general_driver/movtion_module.h::initEncoders()
 */
void encoderInit() {
    // Enable internal pullup resistors for ESP32Encoder
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Attach encoders in half-quadrature mode (using BSP pin definitions)
    // Half-quad uses only one channel (A), with B for direction
    encoderLeft.attachHalfQuad(BSP::Encoder::Left::A_PIN, BSP::Encoder::Left::B_PIN);
    encoderRight.attachHalfQuad(BSP::Encoder::Right::A_PIN, BSP::Encoder::Right::B_PIN);

    // Clear encoder counts
    encoderLeft.setCount(0);
    encoderRight.setCount(0);

    // Initialize timing and counts
    lastTime = micros();
    lastCountLeft = 0;
    lastCountRight = 0;
    speedLeft = 0.0;
    speedRight = 0.0;

    debugLog("ENCODER", "Initialized (half-quad mode)");
    debugPrintf("[ENCODER] Left: GPIO%d/%d, Right: GPIO%d/%d\n",
                BSP::Encoder::Left::A_PIN, BSP::Encoder::Left::B_PIN,
                BSP::Encoder::Right::A_PIN, BSP::Encoder::Right::B_PIN);
    debugPrintf("[ENCODER] Pulses/rev: %d, Wheel dia: %.3fm\n",
                Encoder::PULSES_PER_REV, Robot::WHEEL_DIAMETER_M);
}

/**
 * Get raw encoder counts
 * @param left  Reference to store left encoder count
 * @param right Reference to store right encoder count
 */
void getEncoderCounts(int32_t &left, int32_t &right) {
    left = (int32_t)encoderLeft.getCount();
    right = (int32_t)encoderRight.getCount();
}

/**
 * Calculate and return wheel speeds in m/s
 * Based on reference/general_driver/movtion_module.h::getWheelSpeed()
 *
 * @param left_speed  Reference to store left wheel speed (m/s)
 * @param right_speed Reference to store right wheel speed (m/s)
 *
 * Speed calculation:
 * speed = (delta_pulses / delta_time_seconds) * pulses_to_meters
 */
void getWheelSpeeds(float &left_speed, float &right_speed) {
    unsigned long currentTime = micros();
    int32_t currentCountLeft = (int32_t)encoderLeft.getCount();
    int32_t currentCountRight = (int32_t)encoderRight.getCount();

    // Calculate time delta in seconds
    float deltaTime = (float)(currentTime - lastTime) / 1000000.0;

    // Prevent division by zero (minimum time delta from config)
    if (deltaTime > Encoder::MIN_DELTA_TIME) {
        // Calculate pulse deltas
        int32_t deltaLeft = currentCountLeft - lastCountLeft;
        int32_t deltaRight = currentCountRight - lastCountRight;

        // Calculate speeds in m/s
        // speed = (pulses/time) * (meters/pulse)
        speedLeft = ((float)deltaLeft / deltaTime) * PULSES_TO_METERS;
        speedRight = ((float)deltaRight / deltaTime) * PULSES_TO_METERS;

        // Update last values
        lastCountLeft = currentCountLeft;
        lastCountRight = currentCountRight;
        lastTime = currentTime;
    }

    // Return calculated speeds
    left_speed = speedLeft;
    right_speed = speedRight;
}

/**
 * Reset encoder counts to zero
 */
void resetEncoders() {
    encoderLeft.setCount(0);
    encoderRight.setCount(0);
    lastCountLeft = 0;
    lastCountRight = 0;
    speedLeft = 0.0;
    speedRight = 0.0;
    lastTime = micros();
}
