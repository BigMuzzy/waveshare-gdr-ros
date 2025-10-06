#include "kinematics.h"
#include <Arduino.h>
#include <math.h>

/**
 * Convert twist command to individual wheel speeds
 * Based on differential drive kinematics
 *
 * For a differential drive robot:
 * - Positive linear_x moves forward
 * - Positive angular_z rotates counterclockwise (left wheel slower)
 */
void twistToWheelSpeeds(float linear_x, float angular_z,
                        float &left_speed, float &right_speed) {
    // Differential drive kinematics
    // When turning: inner wheel slows down, outer wheel speeds up
    left_speed = linear_x - (angular_z * WHEEL_BASE / 2.0);
    right_speed = linear_x + (angular_z * WHEEL_BASE / 2.0);
}

/**
 * Convert wheel speeds back to twist command
 * Inverse of twistToWheelSpeeds()
 */
void wheelSpeedsToTwist(float left_speed, float right_speed,
                        float &linear_x, float &angular_z) {
    // Linear velocity is the average of both wheels
    linear_x = (left_speed + right_speed) / 2.0;

    // Angular velocity is the difference divided by wheelbase
    angular_z = (right_speed - left_speed) / WHEEL_BASE;
}

/**
 * Update odometry pose from encoder increments
 * Based on reference/zmoab_ros01/src/motor_control.cpp odometry calculation
 *
 * This implements the standard differential drive odometry model:
 * 1. Convert encoder pulses to linear distances traveled by each wheel
 * 2. Calculate the robot's linear displacement (delta_s) and rotation (delta_theta)
 * 3. Integrate into global pose using the midpoint approximation
 */
void updateOdometry(int32_t left_delta, int32_t right_delta,
                   float &x, float &y, float &theta) {

    // Conversion factor: pulses to meters
    // meters_per_pulse = (π * diameter) / pulses_per_revolution
    const float METERS_PER_PULSE = (M_PI * WHEEL_DIAMETER) / PULSES_PER_REV;

    // Convert encoder pulses to linear distance traveled (meters)
    float left_meters = left_delta * METERS_PER_PULSE;
    float right_meters = right_delta * METERS_PER_PULSE;

    // Calculate robot displacement and rotation
    // delta_s: linear displacement of the robot center
    // delta_theta: change in heading angle
    float delta_s = (left_meters + right_meters) / 2.0;
    float delta_theta = (right_meters - left_meters) / WHEEL_BASE;

    // Update global pose using midpoint method
    // This assumes the robot moves in a straight line at the average heading
    // during this timestep (more accurate than using start or end heading)
    float theta_mid = theta + (delta_theta / 2.0);

    x += delta_s * cos(theta_mid);
    y += delta_s * sin(theta_mid);
    theta += delta_theta;

    // Normalize theta to [-pi, pi] to prevent unbounded growth
    while (theta > M_PI) theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;
}
