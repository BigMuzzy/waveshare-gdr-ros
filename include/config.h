#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/**
 * Waveshare General Driver Board - Configuration File
 *
 * Centralized configuration for ESP32-WROOM-32 based differential drive robot
 * All hardware-specific constants organized in BSP (Board Support Package) namespace
 *
 * Hardware: Waveshare General Driver for ESP32
 * MCU: ESP32-WROOM-32 (NOT S3!)
 * micro-ROS: Serial transport via CP2102 USB-UART bridge
 */

// ============================================================================
// BSP - BOARD SUPPORT PACKAGE (Hardware Pin Definitions)
// ============================================================================
namespace BSP {

    // -------------------------------------------------------------------------
    // UART Pins
    // -------------------------------------------------------------------------
    namespace UART {
        // Main Serial (micro-ROS via CP2102 USB-UART)
        constexpr uint32_t BAUD_RATE = 921600;           // Primary UART baud rate
        constexpr uint32_t INIT_DELAY_MS = 2000;         // Stabilization delay after init

        // Debug Serial (UART2 on separate pins)
        namespace Debug {
            constexpr uint8_t TX_PIN = 5;                // GPIO5 - Debug TX
            constexpr uint8_t RX_PIN = 4;                // GPIO4 - Debug RX
            constexpr uint32_t BAUD_RATE = 115200;       // Debug UART baud rate
            constexpr uint32_t INIT_DELAY_MS = 100;      // Debug init delay
        }

        // Servo Bus (ST3215)
        namespace Servo {
            constexpr uint8_t TX_PIN = 19;               // GPIO19 - Servo TX
            constexpr uint8_t RX_PIN = 18;               // GPIO18 - Servo RX
            constexpr uint32_t BAUD_RATE = 1000000;      // 1Mbaud
        }
    }

    // -------------------------------------------------------------------------
    // I2C Pins & Addresses
    // -------------------------------------------------------------------------
    namespace I2C {
        constexpr uint8_t SDA_PIN = 32;                  // I2C data line
        constexpr uint8_t SCL_PIN = 33;                  // I2C clock line
        constexpr uint32_t FREQUENCY = 400000;           // 400kHz I2C clock
        constexpr uint32_t TIMEOUT_MS = 100;             // Transaction timeout
        constexpr uint8_t MAX_RETRIES = 3;               // Max retry attempts

        // I2C Device Addresses
        namespace Address {
            constexpr uint8_t QMI8658 = 0x6A;           // IMU (or 0x6B)
            constexpr uint8_t AK09918 = 0x0C;           // Magnetometer
            constexpr uint8_t INA219 = 0x42;            // Power monitor
            constexpr uint8_t OLED = 0x3C;              // OLED display
        }
    }

    // -------------------------------------------------------------------------
    // Motor Driver Pins (H-Bridge)
    // -------------------------------------------------------------------------
    namespace MotorDriver {
        // Left Motor (Motor A)
        namespace Left {
            constexpr uint8_t PWM_PIN = 25;             // GPIO25 - Left PWM
            constexpr uint8_t DIR1_PIN = 21;            // GPIO21 - Direction 1
            constexpr uint8_t DIR2_PIN = 17;            // GPIO17 - Direction 2
        }

        // Right Motor (Motor B)
        namespace Right {
            constexpr uint8_t PWM_PIN = 26;             // GPIO26 - Right PWM
            constexpr uint8_t DIR1_PIN = 22;            // GPIO22 - Direction 1
            constexpr uint8_t DIR2_PIN = 23;            // GPIO23 - Direction 2
        }

        // PWM Channels
        constexpr uint8_t PWM_CHANNEL_A = 5;            // Left motor PWM channel
        constexpr uint8_t PWM_CHANNEL_B = 6;            // Right motor PWM channel
    }

    // -------------------------------------------------------------------------
    // Encoder Pins
    // -------------------------------------------------------------------------
    namespace Encoder {
        // Left Encoder
        namespace Left {
            constexpr uint8_t A_PIN = 35;               // GPIO35 - Channel A
            constexpr uint8_t B_PIN = 34;               // GPIO34 - Channel B
        }

        // Right Encoder
        namespace Right {
            constexpr uint8_t A_PIN = 27;               // GPIO27 - Channel A
            constexpr uint8_t B_PIN = 16;               // GPIO16 - Channel B
        }
    }

} // namespace BSP

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
namespace Motor {
    // PWM Configuration
    constexpr uint32_t PWM_FREQUENCY = 100000;           // 100kHz PWM frequency
    constexpr uint8_t PWM_RESOLUTION_BITS = 8;           // 8-bit resolution (0-255)

    // PWM Limits & Thresholds
    constexpr int16_t PWM_MAX = 255;                     // Maximum PWM value
    constexpr int16_t PWM_MIN = -255;                    // Minimum PWM value (negative = reverse)
    constexpr int16_t PWM_ZERO = 0;                      // Zero PWM (stopped)
    constexpr int16_t THRESHOLD_PWM = 10;                // Minimum effective PWM

    // Debug/Display
    constexpr uint32_t KHZ_DIVISOR = 1000;               // Convert Hz to kHz
}

// ============================================================================
// PID CONTROLLER CONFIGURATION
// ============================================================================
namespace PIDConfig {
    // PID Gains (tuned for General Driver Board)
    constexpr double KP = 20.0;                           // Proportional gain
    constexpr double KI = 2000.0;                         // Integral gain
    constexpr double KD = 00.0;                           // Derivative gain

    // Output Limits (PWM range)
    constexpr double OUTPUT_MIN = -255.0;                // Minimum PID output
    constexpr double OUTPUT_MAX = 255.0;                 // Maximum PID output

    // Minimum effective PWM threshold
    constexpr int16_t THRESHOLD = Motor::THRESHOLD_PWM;  // Reuse from Motor namespace

    // PID Update Rate
    constexpr uint32_t COMPUTE_INTERVAL_MS = 10;         // 100Hz PID update rate
}

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================
namespace Encoder {
    // Encoder Specifications
    constexpr int32_t PULSES_PER_REV = 1167;             // Encoder pulses per wheel revolution

    // Timing & Thresholds
    constexpr uint32_t SPEED_UPDATE_US = 10000;          // Speed calculation interval (10ms)
    constexpr uint32_t SPEED_TIMEOUT_MS = 200;           // Speed timeout (consider stopped)
    constexpr float MIN_DELTA_TIME = 0.0001f;            // Minimum time delta (0.1ms) prevent div/0
}

// ============================================================================
// ROBOT PHYSICAL PARAMETERS
// ============================================================================
namespace Robot {
    // Wheel Specifications
    constexpr double WHEEL_DIAMETER_M = 0.068;           // Wheel diameter in meters (68mm)
    constexpr double WHEEL_BASE_M = 0.200;               // Distance between wheels (200mm)

    // Calculated Constants
    constexpr double WHEEL_CIRCUMFERENCE = 3.14159265359 * WHEEL_DIAMETER_M;
    constexpr double METERS_PER_PULSE = WHEEL_CIRCUMFERENCE / Encoder::PULSES_PER_REV;

    // Kinematic Limits
    constexpr float MAX_LINEAR_SPEED = 0.463f;             // Maximum linear speed (m/s)
    constexpr float MAX_ANGULAR_SPEED = 2.0f;            // Maximum angular speed (rad/s)
}

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================
// Enable/disable debug output during runtime (after initialization)
// Set to 0 to disable runtime debug logs, 1 to enable
#define ENABLE_RUNTIME_DEBUG 0
#define ENABLE_INIT_DEBUG 1

// ============================================================================
// ROS2 CONFIGURATION
// ============================================================================
namespace ROS {
    // Node Configuration
    constexpr const char* NODE_NAME = "general_driver";
    constexpr const char* NAMESPACE = "";
    constexpr int DOMAIN_ID = 0;

    // Topic Names
    constexpr const char* TOPIC_ODOM = "/ugv/odom";
    constexpr const char* TOPIC_ENCODER = "/ugv/encoder";
    constexpr const char* TOPIC_CMD_VEL = "/cmd_vel";
    constexpr const char* TOPIC_MOTOR_ENABLE = "/ugv/motor_enable";

    // Frame IDs
    constexpr const char* FRAME_ID_ODOM = "odom";
    constexpr const char* FRAME_ID_BASE_LINK = "base_link";

    // Timing Configuration
    constexpr uint32_t TIMER_PERIOD_MS = 50;             // 20Hz ROS publish rate
    constexpr uint32_t SESSION_TIMEOUT_MS = 1000;        // micro-ROS session sync timeout
    constexpr uint32_t EXECUTOR_SPIN_TIMEOUT_MS = 100;   // Executor spin timeout

    // Array Sizes
    constexpr size_t ENCODER_ARRAY_SIZE = 2;             // 2 encoders (left, right)
    constexpr size_t COVARIANCE_ARRAY_SIZE = 36;         // 6x6 covariance matrix

    // Connection Management
    constexpr uint32_t PING_TIMEOUT_MS = 100;            // Ping timeout
    constexpr uint8_t PING_ATTEMPTS = 1;                 // Ping attempts (waiting)
    constexpr uint8_t PING_ATTEMPTS_CONNECTED = 5;       // Ping attempts (connected)
    constexpr uint32_t AGENT_WAIT_INTERVAL_MS = 500;     // Check interval when waiting
    constexpr uint32_t AGENT_CHECK_INTERVAL_MS = 200;    // Check interval when connected

    // Executor Configuration
    constexpr size_t NUM_SUBSCRIBERS = 2;                // /cmd_vel, /motor_enable
    constexpr size_t NUM_TIMERS = 1;                     // Main publish timer
    constexpr size_t NUM_HANDLES = NUM_SUBSCRIBERS + NUM_TIMERS; // Total = 3
}

// ============================================================================
// IMU CONFIGURATION (QMI8658C)
// ============================================================================
namespace IMU {
    // I2C Configuration
    constexpr uint8_t I2C_ADDRESS = 0x6B;                // QMI8658C I2C address
    constexpr uint32_t I2C_CLOCK = 400000;               // 400kHz I2C clock
    constexpr uint8_t I2C_SDA_PIN = BSP::I2C::SDA_PIN;   // GPIO 32
    constexpr uint8_t I2C_SCL_PIN = BSP::I2C::SCL_PIN;   // GPIO 33

    // Sensor Configuration
    constexpr float ACCEL_RANGE_G = 16.0f;               // ±16g accelerometer range
    constexpr float GYRO_RANGE_DPS = 2048.0f;            // ±2048 degrees/sec gyro range
    constexpr uint16_t ODR_HZ = 1000;                    // 1000Hz output data rate

    // Calibration Parameters
    constexpr uint8_t CALIBRATION_SAMPLES = 50;          // Samples for auto-calibration
    constexpr uint32_t CALIBRATION_DELAY_MS = 10;        // Delay between calibration samples
    constexpr float GRAVITY_MS2 = 9.80665f;              // Standard gravity (m/s²)

    // Conversion Factors (raw sensor value to physical units)
    constexpr float ACCEL_SCALE = ACCEL_RANGE_G / 32768.0f;           // LSB to g, then *9.8 for m/s²
    constexpr float GYRO_SCALE = (GYRO_RANGE_DPS / 32768.0f) *        // LSB to dps
                                 (3.14159265359f / 180.0f);            // dps to rad/s

    // Update Rate (controlled by ROS timer in ros_com.cpp)
    constexpr uint32_t UPDATE_INTERVAL_MS = 50;          // 20Hz IMU publish rate (matches ROS timer)
}

// ============================================================================
// BATTERY MONITOR CONFIGURATION (INA219)
// ============================================================================
namespace Battery {
    // Voltage Thresholds (2S LiPo: 6.0V - 8.4V)
    constexpr float VOLTAGE_MIN = 6.0f;                  // Minimum safe voltage
    constexpr float VOLTAGE_WARN = 6.5f;                 // Low battery warning
    constexpr float VOLTAGE_MAX = 8.4f;                  // Maximum voltage (2S full)

    // Current Limits
    constexpr float CURRENT_MAX_A = 5.0f;                // Maximum current draw

    // Update Rate
    constexpr uint32_t UPDATE_INTERVAL_MS = 1000;        // 1Hz battery monitoring
}

// ============================================================================
// DISPLAY CONFIGURATION (SSD1306 OLED)
// ============================================================================
namespace Display {
    constexpr uint8_t WIDTH = 128;                       // Display width (pixels)
    constexpr uint8_t HEIGHT = 64;                       // Display height (pixels)
    constexpr uint32_t UPDATE_INTERVAL_MS = 500;         // 2Hz display update
}

// ============================================================================
// SERVO CONFIGURATION (ST3215)
// ============================================================================
namespace Servo {
    // Servo IDs
    constexpr uint8_t PAN_ID = 2;                        // Pan servo ID
    constexpr uint8_t TILT_ID = 1;                       // Tilt servo ID

    // PWM Limits
    constexpr uint16_t PWM_MIN = 900;                    // Minimum PWM
    constexpr uint16_t PWM_MID = 1500;                   // Center PWM
    constexpr uint16_t PWM_MAX = 2100;                   // Maximum PWM
}

// ============================================================================
// SAFETY & WATCHDOG
// ============================================================================
namespace Safety {
    constexpr uint32_t CMD_TIMEOUT_MS = 500;             // Command watchdog timeout
    constexpr uint32_t HEARTBEAT_INTERVAL_MS = 3000;     // Status heartbeat interval
    constexpr uint32_t ESTOP_TIMEOUT_MS = 100;           // Emergency stop response time
}

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
namespace Timing {
    constexpr uint32_t MAIN_LOOP_DELAY_MS = 1;           // Main loop delay
    constexpr uint32_t STARTUP_DELAY_MS = 2000;          // System startup delay
}

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

// Validate PWM configuration
static_assert(Motor::PWM_FREQUENCY >= 1000 && Motor::PWM_FREQUENCY <= 200000,
              "PWM frequency must be between 1kHz and 200kHz");
static_assert(Motor::PWM_RESOLUTION_BITS == 8 || Motor::PWM_RESOLUTION_BITS == 10 ||
              Motor::PWM_RESOLUTION_BITS == 12,
              "PWM resolution must be 8, 10, or 12 bits");

// Validate robot dimensions
static_assert(Robot::WHEEL_DIAMETER_M > 0.0 && Robot::WHEEL_DIAMETER_M < 1.0,
              "Wheel diameter must be reasonable (0-1m)");
static_assert(Robot::WHEEL_BASE_M > 0.0 && Robot::WHEEL_BASE_M < 2.0,
              "Wheel base must be reasonable (0-2m)");

// Validate encoder configuration
static_assert(Encoder::PULSES_PER_REV > 0 && Encoder::PULSES_PER_REV < 100000,
              "Encoder pulses per revolution must be positive and reasonable");

// Validate ROS timing
static_assert(ROS::TIMER_PERIOD_MS >= 10 && ROS::TIMER_PERIOD_MS <= 1000,
              "ROS timer period should be between 10ms (100Hz) and 1000ms (1Hz)");

// Validate handle count
static_assert(ROS::NUM_HANDLES == ROS::NUM_SUBSCRIBERS + ROS::NUM_TIMERS,
              "ROS handle count must equal sum of subscribers and timers");

#endif // CONFIG_H
