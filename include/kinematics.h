#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "encoder_handler.h"  // For WHEEL_BASE, WHEEL_DIAMETER, PULSES_PER_REV

/**
 * Differential Drive Kinematics
 *
 * Converts between robot twist commands (linear_x, angular_z) and
 * individual wheel speeds (left, right) for a differential drive robot.
 */

/**
 * Convert twist command to wheel speeds
 *
 * @param linear_x  Forward velocity (m/s)
 * @param angular_z Angular velocity (rad/s, positive = counterclockwise)
 * @param left_speed  Output: left wheel speed (m/s)
 * @param right_speed Output: right wheel speed (m/s)
 *
 * Formula:
 *   left_speed  = linear_x - (angular_z * wheelbase / 2)
 *   right_speed = linear_x + (angular_z * wheelbase / 2)
 */
void twistToWheelSpeeds(float linear_x, float angular_z,
                        float &left_speed, float &right_speed);

/**
 * Convert wheel speeds to twist command
 *
 * @param left_speed  Left wheel speed (m/s)
 * @param right_speed Right wheel speed (m/s)
 * @param linear_x  Output: forward velocity (m/s)
 * @param angular_z Output: angular velocity (rad/s)
 *
 * Formula:
 *   linear_x  = (left_speed + right_speed) / 2
 *   angular_z = (right_speed - left_speed) / wheelbase
 */
void wheelSpeedsToTwist(float left_speed, float right_speed,
                        float &linear_x, float &angular_z);

/**
 * Update odometry from encoder pulse deltas
 *
 * @param left_delta  Change in left encoder count since last update
 * @param right_delta Change in right encoder count since last update
 * @param x     Current X position (m) - will be updated
 * @param y     Current Y position (m) - will be updated
 * @param theta Current heading angle (rad) - will be updated
 *
 * Process:
 * 1. Convert encoder pulses to distance traveled (meters)
 * 2. Calculate robot displacement (delta_s) and rotation (delta_theta)
 * 3. Update pose using current heading for proper 2D integration
 */
void updateOdometry(int32_t left_delta, int32_t right_delta,
                   float &x, float &y, float &theta);

#endif // KINEMATICS_H
