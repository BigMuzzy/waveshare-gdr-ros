#include "encoder_handler.h"
#include "debug_serial.h"
#include <Arduino.h>

// Encoder objects (instantiated here, declared extern in header)
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// Speed calculation variables
static unsigned long last_time_us = 0;
static int32_t last_count_left = 0;
static int32_t last_count_right = 0;
static float speed_left = 0.0;
static float speed_right = 0.0;

// Conversion factor: pulses to meters (from config.h Robot namespace)
static const float PULSES_TO_METERS = Robot::METERS_PER_PULSE;

/**
 * Initialize encoders
 * Based on reference/general_driver/movtion_module.h::initEncoders()
 */
void encoder_init() {
    // Enable internal pullup resistors for ESP32Encoder
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Attach encoders in half-quadrature mode (using BSP pin definitions)
    // Half-quad uses only one channel (A), with B for direction
    encoderLeft.attachHalfQuad(BSP::Encoder::Left::A_PIN, BSP::Encoder::Left::B_PIN);
    encoderRight.attachHalfQuad(BSP::Encoder::Right::B_PIN, BSP::Encoder::Right::A_PIN);

    // Clear encoder counts
    encoderLeft.setCount(0);
    encoderRight.setCount(0);

    // Initialize timing and counts
    last_time_us = micros();
    last_count_left = 0;
    last_count_right = 0;
    speed_left = 0.0;
    speed_right = 0.0;

    debug_log("ENCODER", "Initialized (half-quad mode)");
    debug_printf("[ENCODER] Left: GPIO%d/%d, Right: GPIO%d/%d\n",
                BSP::Encoder::Left::A_PIN, BSP::Encoder::Left::B_PIN,
                BSP::Encoder::Right::A_PIN, BSP::Encoder::Right::B_PIN);
    debug_printf("[ENCODER] Pulses/rev: %d, Wheel dia: %.3fm\n",
                Encoder::PULSES_PER_REV, Robot::WHEEL_DIAMETER_M);
}

/**
 * Get raw encoder counts
 * @param left  Reference to store left encoder count
 * @param right Reference to store right encoder count
 */
void get_encoder_counts(int32_t &left, int32_t &right) {
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
void get_wheel_speeds(float &left_speed, float &right_speed) {
    unsigned long currentTime = micros();
    int32_t currentCountLeft = (int32_t)encoderLeft.getCount();
    int32_t currentCountRight = (int32_t)encoderRight.getCount();

    // Calculate time delta in seconds
    float deltaTime = (float)(currentTime - last_time_us) / 1000000.0;
    //debug_printf("delta t: %f | ", deltaTime);

    // Prevent division by zero (minimum time delta from config)
    if (deltaTime > Encoder::MIN_DELTA_TIME) {
        // Calculate pulse deltas
        int32_t deltaLeft = currentCountLeft - last_count_left;
        int32_t deltaRight = currentCountRight - last_count_right;

        // Calculate speeds in m/s
        // speed = (pulses/time) * (meters/pulse)
        speed_left = ((float)deltaLeft / deltaTime) * PULSES_TO_METERS;
        speed_right = ((float)deltaRight / deltaTime) * PULSES_TO_METERS;

        // Update last values
        last_count_left = currentCountLeft;
        last_count_right = currentCountRight;
        last_time_us = currentTime;
    }

    // Return calculated speeds
    left_speed = speed_left;
    right_speed = speed_right;
    //debug_printf("left speed: %f | right speed %f\n", left_speed, right_speed);
}

/**
 * Reset encoder counts to zero
 */
void reset_encoders() {
    encoderLeft.setCount(0);
    encoderRight.setCount(0);
    last_count_left = 0;
    last_count_right = 0;
    speed_left = 0.0;
    speed_right = 0.0;
    last_time_us = micros();
}
