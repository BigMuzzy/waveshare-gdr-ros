#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

/**
 * @file imu_handler.h
 * @brief QMI8658C 6-axis IMU (Inertial Measurement Unit) interface
 *
 * Provides hardware abstraction for the QMI8658C IMU sensor on the
 * Waveshare General Driver Board. Handles I2C communication, sensor
 * initialization, calibration, and data acquisition.
 *
 * Hardware:
 * - Sensor: QMI8658C (6-axis: 3-axis accelerometer + 3-axis gyroscope)
 * - Interface: I2C @ 400kHz
 * - Address: 0x6B (fixed)
 * - Pins: SDA=GPIO32, SCL=GPIO33
 *
 * Configuration:
 * - Accelerometer: ±16g range, 16-bit resolution
 * - Gyroscope: ±2048 dps range, 16-bit resolution
 * - Output Data Rate: 1000Hz internal sampling
 * - ROS Publishing: 20Hz (controlled externally)
 *
 * Coordinate System:
 * - Follows REP-103 (assumed, requires validation)
 * - X: Forward, Y: Left, Z: Up
 * - Right-handed coordinate frame
 *
 * Usage Example:
 * @code
 * void setup() {
 *     imu_init();           // Initialize I2C and configure sensor
 *     imu_calibrate();      // Auto-calibrate (50 samples, ~500ms)
 * }
 *
 * void loop() {
 *     float accel[3], gyro[3];
 *     if (imu_read(accel, gyro)) {
 *         // accel[0,1,2] = x,y,z acceleration (m/s²)
 *         // gyro[0,1,2] = x,y,z angular velocity (rad/s)
 *     }
 * }
 * @endcode
 *
 * @note Magnetometer NOT supported in this implementation
 * @note All units are SI: m/s² for acceleration, rad/s for angular velocity
 * @note Calibration compensates for static accelerometer bias (gravity)
 *
 * @author Waveshare General Driver Team
 * @date 2025-10-11
 */

#include <Arduino.h>

/**
 * @brief Initialize QMI8658C IMU sensor
 *
 * Performs complete initialization sequence:
 * 1. Initialize I2C bus (400kHz, GPIO32/33)
 * 2. Verify sensor presence (WHO_AM_I check)
 * 3. Software reset sensor
 * 4. Configure accelerometer (±16g, 1000Hz ODR)
 * 5. Configure gyroscope (±2048dps, 1000Hz ODR)
 * 6. Enable both sensors
 *
 * Configuration details:
 * - Accel range: ±16g (ACCEL_SCALE from config.h)
 * - Gyro range: ±2048 degrees/sec (GYRO_SCALE from config.h)
 * - ODR: 1000Hz internal sampling rate
 * - I2C address: 0x6B (from IMU::I2C_ADDRESS in config.h)
 *
 * @return void
 * @note Call this once during setup() before using imu_read()
 * @note Does NOT perform calibration - call imu_calibrate() separately
 * @warning Blocks for ~100ms during initialization
 *
 * @see imu_calibrate() for zero-offset calibration
 * @see is_imu_ready() to check initialization status
 */
void imu_init();

/**
 * @brief Read current IMU sensor data
 *
 * Reads raw sensor values from QMI8658C and converts to physical units:
 * - Accelerometer: Converts 16-bit raw → m/s² (applies ACCEL_SCALE * 9.8)
 * - Gyroscope: Converts 16-bit raw → rad/s (applies GYRO_SCALE)
 * - Applies calibration offsets from imu_calibrate()
 *
 * Data flow:
 * 1. Read 6 bytes accel data (registers 0x35-0x3A)
 * 2. Read 6 bytes gyro data (registers 0x3B-0x40)
 * 3. Convert raw int16_t to physical units
 * 4. Apply calibration offsets
 * 5. Return data in output arrays
 *
 * Coordinate system (REP-103 assumed):
 * - accel[0] = X acceleration (forward, m/s²)
 * - accel[1] = Y acceleration (left, m/s²)
 * - accel[2] = Z acceleration (up, m/s²) - includes gravity
 * - gyro[0] = X angular velocity (roll rate, rad/s)
 * - gyro[1] = Y angular velocity (pitch rate, rad/s)
 * - gyro[2] = Z angular velocity (yaw rate, rad/s)
 *
 * @param[out] accel Array of 3 floats for acceleration data (m/s²)
 * @param[out] gyro Array of 3 floats for angular velocity data (rad/s)
 * @return true if read successful, false on I2C error
 *
 * @note Execution time: ~2ms (I2C transaction time)
 * @note Call at desired publishing rate (20Hz recommended)
 * @warning Returns false if sensor not initialized (call imu_init() first)
 * @warning Accelerometer Z-axis includes gravity (~9.8 m/s² when stationary)
 *
 * @see imu_init() must be called first
 * @see imu_calibrate() for zero-offset calibration
 */
bool imu_read(float accel[3], float gyro[3]);

/**
 * @brief Perform auto-calibration of IMU sensor
 *
 * Computes zero-offset calibration for accelerometer and gyroscope by:
 * 1. Averaging 50 samples (configurable via IMU::CALIBRATION_SAMPLES)
 * 2. Computing mean error for each axis
 * 3. Subtracting gravity from Z-axis accelerometer (9.80665 m/s²)
 * 4. Storing offsets for use in imu_read()
 *
 * Calibration requirements:
 * - Robot MUST be stationary and level during calibration
 * - Z-axis MUST point upward (gravity compensation)
 * - Takes ~500ms (50 samples × 10ms delay)
 *
 * Calibration targets:
 * - Accelerometer X,Y: 0.0 m/s² (no lateral acceleration)
 * - Accelerometer Z: 9.8 m/s² (gravity, will be compensated)
 * - Gyroscope X,Y,Z: 0.0 rad/s (no rotation)
 *
 * After calibration:
 * - Accel offsets compensate for sensor bias
 * - Gyro offsets compensate for drift
 * - Z-axis accel reads ~0.0 when stationary (gravity removed)
 *
 * @return void
 * @note Call once after imu_init(), before normal operation
 * @warning Robot must be completely stationary and level!
 * @warning Blocks for ~500ms during calibration
 * @note Calibration persists until power cycle (not saved to EEPROM)
 *
 * @see IMU::CALIBRATION_SAMPLES in config.h (default: 50)
 * @see IMU::CALIBRATION_DELAY_MS in config.h (default: 10ms)
 * @see IMU::GRAVITY_MS2 in config.h (9.80665 m/s²)
 */
void imu_calibrate();

/**
 * @brief Check if IMU is initialized and ready
 *
 * Verifies that:
 * 1. imu_init() has been called successfully
 * 2. I2C communication is functional
 * 3. Sensor WHO_AM_I register matches expected value
 *
 * Use cases:
 * - Check before calling imu_read()
 * - Verify sensor presence during diagnostics
 * - Detect I2C communication failures
 *
 * @return true if IMU is initialized and responding
 * @return false if not initialized or I2C error
 *
 * @note This is a lightweight check (single I2C read)
 * @note Does NOT verify calibration status
 * @see imu_init() to initialize the sensor
 */
bool is_imu_ready();

#endif // IMU_HANDLER_H
