#include "debug_serial.h"
#include <stdarg.h>

// Use UART2 on custom pins
HardwareSerial DebugSerial(2);

void debugSerialInit(unsigned long baud) {
    // Initialize UART2 on GPIO5 (TX) and GPIO4 (RX)
    DebugSerial.begin(baud, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);

    // Wait a bit for serial to stabilize
    delay(100);

    // Send startup banner
    DebugSerial.println();
    DebugSerial.println("========================================");
    DebugSerial.println("  General Driver Debug Serial");
    DebugSerial.println("  GPIO5 (TX) / GPIO4 (RX)");
    DebugSerial.println("========================================");
    DebugSerial.print("Initialized at ");
    DebugSerial.print(baud);
    DebugSerial.println(" baud");
    DebugSerial.println();
}

void debugPrint(const char* msg) {
    DebugSerial.print(msg);
}

void debugPrint(int val) {
    DebugSerial.print(val);
}

void debugPrint(float val) {
    DebugSerial.print(val);
}

void debugPrintln(const char* msg) {
    DebugSerial.println(msg);
}

void debugPrintln(int val) {
    DebugSerial.println(val);
}

void debugPrintln(float val) {
    DebugSerial.println(val);
}

void debugPrintln() {
    DebugSerial.println();
}

void debugPrintf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    DebugSerial.print(buffer);
}

void debugLog(const char* tag, const char* msg) {
    unsigned long timestamp = millis();
    debugPrintf("[%7lu] [%s] %s\n", timestamp, tag, msg);
}

bool debugSerialAvailable() {
    return DebugSerial.available() > 0;
}
