#include "imu_handler.h"
#include "config.h"
#include "debug_serial.h"
#include <Wire.h>

/**
 * @file imu_handler.cpp
 * @brief QMI8658C IMU sensor driver implementation
 *
 * Implements I2C communication and data processing for the QMI8658C
 * 6-axis IMU (accelerometer + gyroscope) on the Waveshare General Driver Board.
 *
 * Hardware Configuration:
 * - I2C Address: 0x6B (7-bit)
 * - I2C Speed: 400kHz (Fast Mode)
 * - SDA: GPIO32, SCL: GPIO33
 * - Supply: 3.3V
 *
 * Register Map (QMI8658C datasheet):
 * - 0x00: WHO_AM_I (should read 0x05)
 * - 0x02: CTRL1 (serial interface config)
 * - 0x03: CTRL2 (accelerometer config)
 * - 0x04: CTRL3 (gyroscope config)
 * - 0x06: CTRL7 (enable sensors)
 * - 0x35-0x3A: Accel data (X,Y,Z as int16_t little-endian)
 * - 0x3B-0x40: Gyro data (X,Y,Z as int16_t little-endian)
 * - 0x60: RESET (write 0xB0 to software reset)
 *
 * Implementation notes:
 * - Uses Arduino Wire library for I2C
 * - Blocking I2C transactions (~2ms read time)
 * - Calibration offsets stored in static variables
 * - All functions use snake_case naming convention
 *
 * @author Waveshare General Driver Team
 * @date 2025-10-11
 */

// ============================================================================
// QMI8658C Register Definitions
// ============================================================================
namespace QMI8658_Reg {
    constexpr uint8_t WHO_AM_I = 0x00;      // Device ID register (should read 0x05)
    constexpr uint8_t CTRL1 = 0x02;         // Serial interface config
    constexpr uint8_t CTRL2 = 0x03;         // Accelerometer control
    constexpr uint8_t CTRL3 = 0x04;         // Gyroscope control
    constexpr uint8_t CTRL7 = 0x08;         // Enable sensors
    constexpr uint8_t RESET = 0x60;         // Software reset

    constexpr uint8_t ACCEL_X_L = 0x35;     // Accel X-axis low byte
    constexpr uint8_t ACCEL_X_H = 0x36;     // Accel X-axis high byte
    constexpr uint8_t ACCEL_Y_L = 0x37;     // Accel Y-axis low byte
    constexpr uint8_t ACCEL_Y_H = 0x38;     // Accel Y-axis high byte
    constexpr uint8_t ACCEL_Z_L = 0x39;     // Accel Z-axis low byte
    constexpr uint8_t ACCEL_Z_H = 0x3A;     // Accel Z-axis high byte

    constexpr uint8_t GYRO_X_L = 0x3B;      // Gyro X-axis low byte
    constexpr uint8_t GYRO_X_H = 0x3C;      // Gyro X-axis high byte
    constexpr uint8_t GYRO_Y_L = 0x3D;      // Gyro Y-axis low byte
    constexpr uint8_t GYRO_Y_H = 0x3E;      // Gyro Y-axis high byte
    constexpr uint8_t GYRO_Z_L = 0x3F;      // Gyro Z-axis low byte
    constexpr uint8_t GYRO_Z_H = 0x40;      // Gyro Z-axis high byte

    // Expected WHO_AM_I value
    constexpr uint8_t DEVICE_ID = 0x05;

    // Control register values
    constexpr uint8_t RESET_CMD = 0xB0;     // Software reset command
    constexpr uint8_t CTRL1_DEFAULT = 0x60; // Address auto-increment enabled
    constexpr uint8_t CTRL2_16G_1000HZ = 0x33;  // ±16g, 1000Hz ODR
    constexpr uint8_t CTRL3_2048DPS_1000HZ = 0x73; // ±2048dps, 940Hz ODR
    constexpr uint8_t CTRL7_ENABLE_ALL = 0x03;  // Enable accel + gyro
}

// ============================================================================
// Internal State Variables
// ============================================================================
static bool imu_initialized = false;

// Calibration offsets (computed by imu_calibrate())
static float accel_offset[3] = {0.0f, 0.0f, 0.0f};  // m/s²
static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};   // rad/s

// ============================================================================
// Internal I2C Helper Functions
// ============================================================================

/**
 * @brief Write single byte to QMI8658C register
 * @param reg Register address
 * @param value Value to write
 * @return true if write successful, false on I2C error
 */
static bool write_register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU::I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    uint8_t error = Wire.endTransmission();
    return (error == 0);
}

/**
 * @brief Read single byte from QMI8658C register
 * @param reg Register address
 * @param value Pointer to store read value
 * @return true if read successful, false on I2C error
 */
static bool read_register(uint8_t reg, uint8_t* value) {
    Wire.beginTransmission(IMU::I2C_ADDRESS);
    Wire.write(reg);
    uint8_t error = Wire.endTransmission(false); // Repeated start
    if (error != 0) return false;

    uint8_t bytes_read = Wire.requestFrom(IMU::I2C_ADDRESS, (uint8_t)1);
    if (bytes_read != 1) return false;

    *value = Wire.read();
    return true;
}

/**
 * @brief Read multiple bytes from QMI8658C (auto-increment)
 * @param reg Starting register address
 * @param buffer Buffer to store read data
 * @param length Number of bytes to read
 * @return true if read successful, false on I2C error
 */
static bool read_registers(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(IMU::I2C_ADDRESS);
    Wire.write(reg);
    uint8_t error = Wire.endTransmission(false); // Repeated start
    if (error != 0) return false;

    uint8_t bytes_read = Wire.requestFrom(IMU::I2C_ADDRESS, length);
    if (bytes_read != length) return false;

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

/**
 * @brief Convert two bytes (little-endian) to signed 16-bit integer
 * @param low Low byte
 * @param high High byte
 * @return Signed 16-bit value
 */
static inline int16_t bytes_to_int16(uint8_t low, uint8_t high) {
    return (int16_t)((high << 8) | low);
}

// ============================================================================
// Public API Implementation
// ============================================================================

void imu_init() {
    debug_log("IMU", "Initializing QMI8658C...");

    // 1. Initialize I2C bus
    Wire.begin(IMU::I2C_SDA_PIN, IMU::I2C_SCL_PIN);
    Wire.setClock(IMU::I2C_CLOCK);
    delay(10); // Allow I2C bus to stabilize

    debug_print("I2C initialized: SDA=GPIO");
    debug_print((int)IMU::I2C_SDA_PIN);
    debug_print(", SCL=GPIO");
    debug_print((int)IMU::I2C_SCL_PIN);
    debug_print(", Clock=");
    debug_print((int)(IMU::I2C_CLOCK / 1000));
    debug_println("kHz");

    // 2. Verify sensor presence (WHO_AM_I check)
    uint8_t who_am_i = 0;
    debug_print("Reading WHO_AM_I from address 0x");
    debug_print((int)IMU::I2C_ADDRESS);
    debug_println("...");

    if (!read_register(QMI8658_Reg::WHO_AM_I, &who_am_i)) {
        debug_log("IMU", "ERROR: I2C communication failed!");
        debug_println("Possible issues:");
        debug_println("  - Wrong I2C address (check 0x6A vs 0x6B)");
        debug_println("  - Sensor not powered");
        debug_println("  - SDA/SCL wiring incorrect");
        debug_println("  - Pull-up resistors missing");
        imu_initialized = false;
        return;
    }

    debug_print("WHO_AM_I register read: 0x");
    debug_println((int)who_am_i);

    if (who_am_i != QMI8658_Reg::DEVICE_ID) {
        debug_log("IMU", "ERROR: Wrong device ID!");
        debug_print("Expected: 0x05, Got: 0x");
        debug_println((int)who_am_i);
        imu_initialized = false;
        return;
    }

    debug_log("IMU", "Device ID verified: 0x05");

    // 3. Software reset
    debug_log("IMU", "Performing software reset...");
    write_register(QMI8658_Reg::RESET, QMI8658_Reg::RESET_CMD);
    delay(50); // Wait for reset to complete

    // 4. Configure CTRL1 (serial interface, auto-increment enabled)
    write_register(QMI8658_Reg::CTRL1, QMI8658_Reg::CTRL1_DEFAULT);
    delay(1);

    // 5. Configure accelerometer (±16g, 1000Hz ODR)
    debug_log("IMU", "Configuring accelerometer (±16g, 1000Hz)...");
    if(!write_register(QMI8658_Reg::CTRL2, QMI8658_Reg::CTRL2_16G_1000HZ)) {
        debug_log("IMU", "ERROR: Failed to write CTRL2!");
        imu_initialized = false;
        return;
    }
    delay(1);

    // 6. Configure gyroscope (±2048dps, 1000Hz ODR)
    debug_log("IMU", "Configuring gyroscope (±2048dps, 1000Hz)...");
    write_register(QMI8658_Reg::CTRL3, QMI8658_Reg::CTRL3_2048DPS_1000HZ);
    delay(1);

    // 7. Enable both sensors
    debug_log("IMU", "Enabling accelerometer and gyroscope...");
    write_register(QMI8658_Reg::CTRL7, QMI8658_Reg::CTRL7_ENABLE_ALL);
    delay(10); // Allow sensors to start

    // 8. Verify configuration by reading back control registers
    uint8_t ctrl2_readback = 0, ctrl3_readback = 0, ctrl7_readback = 0;
    read_register(QMI8658_Reg::CTRL2, &ctrl2_readback);
    read_register(QMI8658_Reg::CTRL3, &ctrl3_readback);
    read_register(QMI8658_Reg::CTRL7, &ctrl7_readback);
    // In imu_init(), after configuration
    debug_println("Register readback verification:");

    uint8_t ctrl2_val = 0;
    uint8_t ctrl3_val = 0;
    uint8_t ctrl7_val = 0;

    if (read_register(QMI8658_Reg::CTRL2, &ctrl2_val)) {
        debug_print("  CTRL2 (Accel): ");
        debug_println(ctrl2_val);  // Print the VALUE, not the pointer
    } else {
        debug_println("  CTRL2: READ FAILED!");
    }

    if (read_register(QMI8658_Reg::CTRL3, &ctrl3_val)) {
        debug_print("  CTRL3 (Gyro): ");
        debug_println(ctrl3_val);
    } else {
        debug_println("  CTRL3: READ FAILED!");
    }

    if (read_register(QMI8658_Reg::CTRL7, &ctrl7_val)) {
        debug_print("  CTRL7 (Enable): ");
        debug_println(ctrl7_val);
    } else {
        debug_println("  CTRL7: READ FAILED!");
    }

    imu_initialized = true;
    debug_log("IMU", "Initialization complete!");
    debug_println();
}

bool imu_read(float accel[3], float gyro[3]) {
    if (!imu_initialized) {
        debug_log("IMU", "Read failed: not initialized");
        return false;
    }

    // Read all sensor data in one transaction (12 bytes: 6 accel + 6 gyro)
    // This is more efficient than separate reads
    uint8_t raw_data[12];
    if (!read_registers(QMI8658_Reg::ACCEL_X_L, raw_data, 12)) {
        debug_log("IMU", "Read failed: I2C error");
        return false;
    }

    // Parse accelerometer data (bytes 0-5)
    int16_t accel_raw[3];
    accel_raw[0] = bytes_to_int16(raw_data[0], raw_data[1]); // X
    accel_raw[1] = bytes_to_int16(raw_data[2], raw_data[3]); // Y
    accel_raw[2] = bytes_to_int16(raw_data[4], raw_data[5]); // Z

    // Parse gyroscope data (bytes 6-11)
    int16_t gyro_raw[3];
    gyro_raw[0] = bytes_to_int16(raw_data[6], raw_data[7]);   // X
    gyro_raw[1] = bytes_to_int16(raw_data[8], raw_data[9]);   // Y
    gyro_raw[2] = bytes_to_int16(raw_data[10], raw_data[11]); // Z

    // Convert accelerometer to m/s² and apply calibration
    // Formula: (raw * ACCEL_SCALE * gravity) - offset
    for (int i = 0; i < 3; i++) {
        accel[i] = (accel_raw[i] * IMU::ACCEL_SCALE * IMU::GRAVITY_MS2) - accel_offset[i];
    }

    // Convert gyroscope to rad/s and apply calibration
    // Formula: (raw * GYRO_SCALE) - offset
    for (int i = 0; i < 3; i++) {
        gyro[i] = (gyro_raw[i] * IMU::GYRO_SCALE) - gyro_offset[i];
    }

    return true;
}

void imu_calibrate() {
    if (!imu_initialized) {
        debug_log("IMU", "ERROR: Cannot calibrate - not initialized!");
        return;
    }

    debug_log("IMU", "Starting calibration...");
    debug_log("IMU", "Robot must be stationary and level!");
    delay(1000); // Give user time to stabilize robot

    // Accumulate samples for averaging
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    uint8_t valid_samples = 0;

    debug_print("Collecting ");
    debug_print(IMU::CALIBRATION_SAMPLES);
    debug_println(" samples...");
    debug_println("Sample | Accel X    Y       Z      | Gyro X     Y      Z     | Raw bytes");
    debug_println("-------|---------------------------|----------------------|------------");

    for (uint8_t i = 0; i < IMU::CALIBRATION_SAMPLES; i++) {
        float accel[3], gyro[3];

        // Read raw data for debugging
        uint8_t raw_data[12];
        read_registers(QMI8658_Reg::ACCEL_X_L, raw_data, 12);
        
        int16_t accel_raw[3];
        accel_raw[0] = bytes_to_int16(raw_data[0], raw_data[1]);
        accel_raw[1] = bytes_to_int16(raw_data[2], raw_data[3]);
        accel_raw[2] = bytes_to_int16(raw_data[4], raw_data[5]);
        
        if (i % 5 == 4) {
            debug_print("  Raw Z: ");
            debug_print(accel_raw[2]);  // Should be around -2048 for -1g
            debug_print(" | Scaled Z: ");
            debug_println(accel_raw[2] * IMU::ACCEL_SCALE * IMU::GRAVITY_MS2);
        }

        // Also read raw bytes for first sample to debug
        bool has_raw = false;
        if (i == 0) {
            has_raw = read_registers(QMI8658_Reg::ACCEL_X_L, raw_data, 12);
        }

        // Read raw data (without applying current offsets)
        // We temporarily read with offsets, then we'll recompute them
        if (imu_read(accel, gyro)) {
            // Add current offsets back to get raw values
            float raw_accel[3], raw_gyro[3];
            for (int axis = 0; axis < 3; axis++) {
                raw_accel[axis] = accel[axis] + accel_offset[axis];
                raw_gyro[axis] = gyro[axis] + gyro_offset[axis];
                accel_sum[axis] += raw_accel[axis];
                gyro_sum[axis] += raw_gyro[axis];
            }
            valid_samples++;

            // Print raw sample values (every 5th sample to reduce clutter)
            if ((i + 1) % 5 == 0) {
                debug_print(" ");
                debug_print((int)(i + 1));
                debug_print("    | ");
                debug_print(raw_accel[0]);
                debug_print(" ");
                debug_print(raw_accel[1]);
                debug_print(" ");
                debug_print(raw_accel[2]);
                debug_print(" | ");
                debug_print(raw_gyro[0]);
                debug_print(" ");
                debug_print(raw_gyro[1]);
                debug_print(" ");
                debug_print(raw_gyro[2]);

                // Print raw bytes for first sample
                if (i == 0 && has_raw) {
                    debug_print(" | ");
                    for (int j = 0; j < 12; j++) {
                        if (raw_data[j] < 16) debug_print("0");
                        debug_print((int)raw_data[j]);
                        debug_print(" ");
                    }
                }
                debug_println("");
            }
        } else {
            debug_log("IMU", "WARNING: Failed to read sample during calibration!");
        }

        delay(IMU::CALIBRATION_DELAY_MS);
    }
    debug_println();

    if (valid_samples < IMU::CALIBRATION_SAMPLES / 2) {
        debug_log("IMU", "ERROR: Calibration failed - too few valid samples!");
        return;
    }

    // Compute average offsets
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = accel_sum[i] / valid_samples;
        gyro_offset[i] = gyro_sum[i] / valid_samples;
    }

    // Compensate for gravity on Z-axis
    // When stationary and level, Z-axis should read +1g (9.8 m/s²)
    // We want calibrated Z to read ~0 when stationary, so subtract gravity
    accel_offset[2] -= IMU::GRAVITY_MS2;

    debug_log("IMU", "Calibration complete!");
    debug_print("Accel offsets (m/s²): X=");
    debug_print(accel_offset[0]);
    debug_print(", Y=");
    debug_print(accel_offset[1]);
    debug_print(", Z=");
    debug_println(accel_offset[2]);

    debug_print("Gyro offsets (rad/s): X=");
    debug_print(gyro_offset[0]);
    debug_print(", Y=");
    debug_print(gyro_offset[1]);
    debug_print(", Z=");
    debug_println(gyro_offset[2]);
    debug_println();
}

bool is_imu_ready() {
    if (!imu_initialized) {
        return false;
    }

    // Verify I2C communication is still working
    uint8_t who_am_i = 0;
    if (!read_register(QMI8658_Reg::WHO_AM_I, &who_am_i)) {
        return false;
    }

    return (who_am_i == QMI8658_Reg::DEVICE_ID);
}
