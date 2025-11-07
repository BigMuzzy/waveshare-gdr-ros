#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include "config.h"

/**
 * @file debug_serial.h
 * @brief Debug serial interface using dedicated UART2
 *
 * Provides debug logging on separate UART pins to avoid conflicts with
 * micro-ROS Serial transport. Uses GPIO5 (TX) and GPIO4 (RX).
 */

// Debug serial object - uses hardware UART2
extern HardwareSerial DebugSerial;

/**
 * @brief Initialize debug serial port
 * @param baud Baud rate (default: 115200 from config.h)
 *
 * Configures UART2 on dedicated GPIO pins for debug output.
 * Safe to call even when main Serial is used by micro-ROS.
 */
void debug_serial_init(unsigned long baud = BSP::UART::Debug::BAUD_RATE);

/**
 * @brief Print debug message (same as Serial.print)
 */
void debug_print(const char* msg);
void debug_print(int val);
void debug_print(float val);

/**
 * @brief Print debug message with newline (same as Serial.println)
 */
void debug_println(const char* msg);
void debug_println(int val);
void debug_println(float val);
void debug_println();

/**
 * @brief Printf-style formatted debug output
 * @param format Printf format string
 * @param ... Variable arguments
 *
 * Example: debug_printf("Encoder: L=%d R=%d\n", left, right);
 */
void debug_printf(const char* format, ...);

/**
 * @brief Log message with timestamp and tag
 * @param tag Category tag (e.g., "MOTOR", "IMU", "PID")
 * @param msg Message to log
 *
 * Outputs: [timestamp] [TAG] message
 * Example: debug_log("ENCODER", "Left overflow detected");
 */
void debug_log(const char* tag, const char* msg);

/**
 * @brief Log message with timestamp and tag (initialization phase only)
 * @param tag Category tag (e.g., "INIT", "MOTOR", "IMU")
 * @param msg Message to log
 *
 * Always outputs during initialization phase regardless of ENABLE_RUNTIME_DEBUG setting
 */
void debug_log_init(const char* tag, const char* msg);

/**
 * @brief Check if debug serial has data available
 * @return true if data available to read
 */
bool debug_serial_available();

// Conditional debug macros - runtime debug can be disabled via config.h
#include "config.h"

#if ENABLE_RUNTIME_DEBUG
    #define DEBUG_LOG(tag, msg) debug_log(tag, msg)
    #define DEBUG_PRINT(x) debug_print(x)
    #define DEBUG_PRINTLN(x) debug_println(x)
    #define DEBUG_PRINTF(...) debug_printf(__VA_ARGS__)
#else
    #define DEBUG_LOG(tag, msg) ((void)0)
    #define DEBUG_PRINT(x) ((void)0)
    #define DEBUG_PRINTLN(x) ((void)0)
    #define DEBUG_PRINTF(...) ((void)0)
#endif

// Initialization logs always enabled
#define DEBUG_LOG_INIT(tag, msg) debug_log_init(tag, msg)

#endif // DEBUG_SERIAL_H
