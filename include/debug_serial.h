#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include "config.h"

// Use GPIO pins from config.h: Debug::TX_PIN, Debug::RX_PIN
// Use baud rate from config.h: Debug::BAUD_RATE

// Debug serial object - uses hardware UART2
extern HardwareSerial DebugSerial;

/**
 * Initialize debug serial port
 * @param baud Baud rate (default from config.h: Debug::BAUD_RATE)
 */
void debugSerialInit(unsigned long baud = Debug::BAUD_RATE);

/**
 * Print debug message (same as Serial.print)
 */
void debugPrint(const char* msg);
void debugPrint(int val);
void debugPrint(float val);

/**
 * Print debug message with newline (same as Serial.println)
 */
void debugPrintln(const char* msg);
void debugPrintln(int val);
void debugPrintln(float val);
void debugPrintln();

/**
 * Printf-style formatted debug output
 * @param format Printf format string
 * @param ... Variable arguments
 *
 * Example: debugPrintf("Encoder: L=%d R=%d\n", left, right);
 */
void debugPrintf(const char* format, ...);

/**
 * Log with timestamp
 * @param tag Category tag (e.g., "MOTOR", "IMU", "PID")
 * @param msg Message to log
 *
 * Example: debugLog("ENCODER", "Left overflow detected");
 */
void debugLog(const char* tag, const char* msg);

/**
 * Check if debug serial is available (for conditional debugging)
 */
bool debugSerialAvailable();

#endif // DEBUG_SERIAL_H
