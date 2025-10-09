#include "debug_serial.h"
#include <stdarg.h>

// Use UART2 on custom pins (from config.h BSP namespace)
HardwareSerial DebugSerial(2);

void debugSerialInit(unsigned long baud) {
    // Initialize UART2 on GPIO pins from BSP::UART::Debug namespace
    DebugSerial.begin(baud, SERIAL_8N1, BSP::UART::Debug::RX_PIN, BSP::UART::Debug::TX_PIN);

    // Wait for serial to stabilize (delay from config.h)
    delay(BSP::UART::Debug::INIT_DELAY_MS);

    // Send startup banner
    DebugSerial.println();
    DebugSerial.println("========================================");
    DebugSerial.println("  General Driver Debug Serial");
    DebugSerial.printf("  GPIO%d (TX) / GPIO%d (RX)\n",
                       BSP::UART::Debug::TX_PIN, BSP::UART::Debug::RX_PIN);
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
