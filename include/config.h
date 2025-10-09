#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/**
 * Waveshare General Driver Board - Configuration File
 *
 * Centralized configuration for ESP32-WROOM-32 based differential drive robot
 * All magic numbers should be defined here for easy customization
 *
 * Hardware: Waveshare General Driver for ESP32
 * MCU: ESP32-WROOM-32 (NOT S3!)
 * micro-ROS: Serial transport via CP2102 USB-UART bridge
 */

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================
namespace UARTConfig {
    constexpr uint32_t BAUD_RATE = 115200;           // CP2102 USB-UART baud rate
    constexpr uint32_t INIT_DELAY_MS = 2000;         // Delay after serial init for stabilization
}

// ============================================================================
// DEBUG SERIAL (UART2 on GPIO5/4)
// ============================================================================
namespace Debug {
    constexpr uint8_t TX_PIN = 5;                    // GPIO5 - TX to USB-UART adapter RX
    constexpr uint8_t RX_PIN = 4;                    // GPIO4 - RX from USB-UART adapter TX
    constexpr uint32_t BAUD_RATE = 115200;           // Debug serial baud rate
    constexpr uint32_t INIT_DELAY_MS = 100;          // Delay for debug serial stabilization
}

// ============================================================================
// MOTOR CONTROL (H-Bridge)
// ============================================================================
namespace Motor {
    // Pin definitions - Left Motor (Motor A)
    constexpr uint8_t PWMA_PIN = 25;                 // Left motor PWM control
    constexpr uint8_t AIN1_PIN = 21;                 // Left motor direction 1
    constexpr uint8_t AIN2_PIN = 17;                 // Left motor direction 2

    // Pin definitions - Right Motor (Motor B)
    constexpr uint8_t PWMB_PIN = 26;                 // Right motor PWM control
    constexpr uint8_t BIN1_PIN = 22;                 // Right motor direction 1
    constexpr uint8_t BIN2_PIN = 23;                 // Right motor direction 2

    // PWM configuration
    constexpr uint32_t PWM_FREQUENCY = 100000;       // 100kHz PWM frequency
    constexpr uint8_t PWM_CHANNEL_A = 5;             // Left motor PWM channel
    constexpr uint8_t PWM_CHANNEL_B = 6;             // Right motor PWM channel
    constexpr uint8_t PWM_RESOLUTION_BITS = 8;       // 8-bit resolution (0-255)

    // PWM limits
    constexpr int16_t PWM_MAX = 255;                 // Maximum PWM value
    constexpr int16_t PWM_MIN = -255;                // Minimum PWM value (negative = reverse)
    constexpr int16_t PWM_ZERO = 0;                  // Zero PWM (stopped)
    constexpr int16_t PWM_DEADZONE = 1;              // Minimum PWM to ignore (motor won't turn)
    constexpr int16_t THRESHOLD_PWM = 23;            // Minimum effective PWM threshold

    // Speed limits
    constexpr float MAX_SPEED_MPS = 2.0f;            // Maximum speed (m/s)
    constexpr float MIN_SPEED_MPS = -2.0f;           // Minimum speed (m/s)

    // Conversion factor for debug output
    constexpr uint32_t KHZ_DIVISOR = 1000;           // Convert Hz to kHz for display
}

// ============================================================================
// PID CONTROLLER
// ============================================================================
namespace PIDConfig {
    // PID gains (tuned for General Driver Board)
    constexpr double KP = 20.0;                      // Proportional gain
    constexpr double KI = 2000.0;                    // Integral gain
    constexpr double KD = 0.0;                       // Derivative gain

    // Output limits (PWM range)
    constexpr double OUTPUT_MIN = -255.0;            // Minimum PID output
    constexpr double OUTPUT_MAX = 255.0;             // Maximum PID output

    // Minimum effective PWM threshold
    constexpr int16_t THRESHOLD = Motor::THRESHOLD_PWM; // Below this, motor won't move effectively

    // PID compute rate
    constexpr uint32_t COMPUTE_INTERVAL_MS = 10;     // 100Hz PID update rate
}

// ============================================================================
// ENCODERS
// ============================================================================
namespace Encoder {
    // Pin definitions - Left Encoder
    constexpr uint8_t LEFT_A_PIN = 35;               // Left encoder channel A (AENCA)
    constexpr uint8_t LEFT_B_PIN = 34;               // Left encoder channel B (AENCB)

    // Pin definitions - Right Encoder
    constexpr uint8_t RIGHT_A_PIN = 27;              // Right encoder channel A (BENCA)
    constexpr uint8_t RIGHT_B_PIN = 16;              // Right encoder channel B (BENCB)

    // Encoder specifications
    constexpr int32_t PULSES_PER_REV = 1170;         // Encoder pulses per wheel revolution

    // Timing
    constexpr uint32_t SPEED_UPDATE_US = 10000;      // Speed calculation interval (10ms)
    constexpr uint32_t SPEED_TIMEOUT_MS = 200;       // Speed timeout (consider stopped)
    constexpr float MIN_DELTA_TIME = 0.0001f;        // Minimum time delta (0.1ms) to prevent div by zero
}

// ============================================================================
// ROBOT PHYSICAL PARAMETERS
// ============================================================================
namespace Robot {
    // Wheel specifications
    constexpr double WHEEL_DIAMETER_M = 0.065;       // Wheel diameter in meters (65mm)
    constexpr double WHEEL_BASE_M = 0.200;           // Distance between wheels in meters (200mm)

    // Calculated constants (using M_PI from math.h to avoid conflict with Arduino PI macro)
    constexpr double WHEEL_CIRCUMFERENCE = 3.14159265359 * WHEEL_DIAMETER_M;
    constexpr double METERS_PER_PULSE = WHEEL_CIRCUMFERENCE / Encoder::PULSES_PER_REV;

    // Kinematic limits
    constexpr float MAX_LINEAR_SPEED = 0.5f;         // Maximum linear speed (m/s)
    constexpr float MAX_ANGULAR_SPEED = 2.0f;        // Maximum angular speed (rad/s)
}

// ============================================================================
// ROS2 CONFIGURATION
// ============================================================================
namespace ROS {
    // Node configuration
    constexpr const char* NODE_NAME = "general_driver";
    constexpr const char* NAMESPACE = "";
    constexpr int DOMAIN_ID = 0;

    // Topic names
    constexpr const char* TOPIC_ODOM = "/ugv/odom";
    constexpr const char* TOPIC_ENCODER = "/ugv/encoder";
    constexpr const char* TOPIC_CMD_VEL = "/cmd_vel";
    constexpr const char* TOPIC_MOTOR_ENABLE = "/ugv/motor_enable";

    // Frame IDs
    constexpr const char* FRAME_ID_ODOM = "odom";
    constexpr const char* FRAME_ID_BASE_LINK = "base_link";

    // Timing
    constexpr uint32_t TIMER_PERIOD_MS = 50;         // 20Hz ROS publish rate
    constexpr uint32_t SESSION_TIMEOUT_MS = 1000;    // micro-ROS session sync timeout
    constexpr uint32_t EXECUTOR_SPIN_TIMEOUT_MS = 100; // Executor spin timeout

    // Array sizes
    constexpr size_t ENCODER_ARRAY_SIZE = 2;         // 2 encoders (left, right)
    constexpr size_t COVARIANCE_ARRAY_SIZE = 36;     // 6x6 covariance matrix

    // Connection management
    constexpr uint32_t PING_INTERVAL_MS = 200;       // Agent ping interval
    constexpr uint32_t PING_TIMEOUT_MS = 100;        // Ping timeout
    constexpr uint8_t PING_ATTEMPTS = 1;             // Ping attempts per check
    constexpr uint32_t AGENT_CHECK_INTERVAL_MS = 200; // Check agent every 200ms when connected
    constexpr uint32_t AGENT_WAIT_INTERVAL_MS = 500;  // Check agent every 500ms when waiting
    constexpr uint8_t PING_ATTEMPTS_CONNECTED = 5;    // More attempts when verifying connection

    // Executor handles
    constexpr size_t NUM_SUBSCRIBERS = 2;            // /cmd_vel, /motor_enable
    constexpr size_t NUM_TIMERS = 1;                 // Main publish timer
    constexpr size_t NUM_HANDLES = NUM_SUBSCRIBERS + NUM_TIMERS; // Total = 3
}

// ============================================================================
// I2C PERIPHERALS
// ============================================================================
namespace I2C {
    // I2C bus pins
    constexpr uint8_t SDA_PIN = 32;                  // I2C data line
    constexpr uint8_t SCL_PIN = 33;                  // I2C clock line
    constexpr uint32_t FREQUENCY = 400000;           // 400kHz I2C clock

    // Device addresses
    constexpr uint8_t QMI8658_ADDR = 0x6A;          // IMU address (or 0x6B)
    constexpr uint8_t AK09918_ADDR = 0x0C;          // Magnetometer address
    constexpr uint8_t INA219_ADDR = 0x42;           // Power monitor address
    constexpr uint8_t OLED_ADDR = 0x3C;             // OLED display address

    // I2C communication
    constexpr uint32_t TIMEOUT_MS = 100;             // I2C transaction timeout
    constexpr uint8_t MAX_RETRIES = 3;               // Maximum I2C retry attempts
}

// ============================================================================
// IMU (QMI8658C + AK09918)
// ============================================================================
namespace IMU {
    // Calibration
    constexpr uint8_t CALIBRATION_SAMPLES = 50;      // Number of samples for auto-calibration
    constexpr uint32_t CALIBRATION_DELAY_MS = 10;    // Delay between calibration samples

    // Physical constants
    constexpr double GRAVITY_MPS2 = 9.80665;         // Standard gravity (m/s²)
    constexpr double GRAVITY_OFFSET = 980.0;         // Gravity offset for Z-axis calibration

    // Update rate
    constexpr uint32_t UPDATE_INTERVAL_MS = 20;      // 50Hz IMU update rate
}

// ============================================================================
// BATTERY MONITOR (INA219)
// ============================================================================
namespace Battery {
    // Voltage thresholds (2S LiPo typical: 6.0V - 8.4V)
    constexpr float VOLTAGE_MIN = 6.0f;              // Minimum safe voltage
    constexpr float VOLTAGE_WARN = 6.5f;             // Low battery warning threshold
    constexpr float VOLTAGE_MAX = 8.4f;              // Maximum voltage (fully charged 2S)

    // Current limits
    constexpr float CURRENT_MAX_A = 5.0f;            // Maximum current draw

    // Update rate
    constexpr uint32_t UPDATE_INTERVAL_MS = 1000;    // 1Hz battery monitoring
}

// ============================================================================
// OLED DISPLAY (SSD1306)
// ============================================================================
namespace Display {
    constexpr uint8_t WIDTH = 128;                   // Display width in pixels
    constexpr uint8_t HEIGHT = 64;                   // Display height in pixels
    constexpr uint32_t UPDATE_INTERVAL_MS = 500;     // 2Hz display update
}

// ============================================================================
// SERVO CONTROL (ST3215)
// ============================================================================
namespace Servo {
    // Serial bus pins
    constexpr uint8_t RX_PIN = 18;                   // Servo UART RX
    constexpr uint8_t TX_PIN = 19;                   // Servo UART TX
    constexpr uint32_t BAUD_RATE = 1000000;          // 1Mbaud

    // Servo IDs
    constexpr uint8_t PAN_ID = 2;                    // Pan servo ID
    constexpr uint8_t TILT_ID = 1;                   // Tilt servo ID

    // PWM limits
    constexpr uint16_t PWM_MIN = 900;                // Minimum PWM value
    constexpr uint16_t PWM_MID = 1500;               // Center PWM value
    constexpr uint16_t PWM_MAX = 2100;               // Maximum PWM value
}

// ============================================================================
// SAFETY & WATCHDOG
// ============================================================================
namespace Safety {
    constexpr uint32_t CMD_TIMEOUT_MS = 500;         // Command watchdog timeout
    constexpr uint32_t HEARTBEAT_INTERVAL_MS = 3000; // Status heartbeat interval
    constexpr uint32_t ESTOP_TIMEOUT_MS = 100;       // Emergency stop response time
}

// ============================================================================
// TIMING
// ============================================================================
namespace Timing {
    constexpr uint32_t MAIN_LOOP_DELAY_MS = 1;       // Main loop delay (yield to other tasks)
    constexpr uint32_t STARTUP_DELAY_MS = 2000;      // System startup delay
}

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

// Validate PWM frequency is reasonable
static_assert(Motor::PWM_FREQUENCY >= 1000 && Motor::PWM_FREQUENCY <= 200000,
              "PWM frequency must be between 1kHz and 200kHz");

// Validate PWM resolution
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
