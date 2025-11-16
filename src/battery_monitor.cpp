/**
 * @file battery_monitor.cpp
 * @brief Battery voltage and current monitoring implementation
 *
 * Uses INA219 power monitor to track 3S LiPo battery status.
 * Based on reference/general_driver/General_Driver/battery_ctrl.h
 */

#include "battery_monitor.h"
#include "config.h"
#include "debug_serial.h"
#include <INA219_WE.h>

// INA219 sensor object
static INA219_WE ina219(Battery::I2C_ADDRESS);

// Initialization flag
static bool battery_initialized = false;

bool battery_init() {
    DEBUG_LOG_INIT("BATTERY", "Initializing INA219...");

    // Initialize INA219 (Wire.begin() already called by IMU init)
    if (!ina219.init()) {
        DEBUG_LOG_INIT("BATTERY", "INA219 not found at 0x42!");
        battery_initialized = false;
        return false;
    }

    // Configure INA219 for 3S LiPo monitoring
    ina219.setADCMode(INA219_BIT_MODE_9);       // 9-bit resolution (faster, adequate)
    ina219.setPGain(INA219_PG_320);             // ±320mV shunt voltage range
    ina219.setBusRange(INA219_BRNG_16);         // 16V bus voltage range (for 3S: 9-12.6V)
    ina219.setShuntSizeInOhms(Battery::SHUNT_RESISTOR_OHMS);  // 0.01Ω shunt

    battery_initialized = true;

    DEBUG_LOG_INIT("BATTERY", "INA219 configured for 3S LiPo");
    debug_printf("[BATTERY] Range: %.1fV-%.1fV, Shunt: %.3fΩ\n",
                 Battery::VOLTAGE_MIN, Battery::VOLTAGE_MAX,
                 Battery::SHUNT_RESISTOR_OHMS);

    return true;
}

bool battery_read(float &voltage, float &current, float &power) {
    if (!battery_initialized) {
        return false;
    }

    // Read measurements from INA219
    float shunt_voltage_mv = ina219.getShuntVoltage_mV();
    float bus_voltage_v = ina219.getBusVoltage_V();

    // Calculate load voltage (bus + shunt drop)
    voltage = bus_voltage_v + (shunt_voltage_mv / 1000.0f);

    // Read current (in mA, convert to A)
    current = ina219.getCurrent_mA() / 1000.0f;

    // Calculate power (in W)
    power = voltage * current;

    // Check for overflow (measurement out of range)
    if (ina219.getOverflow()) {
        DEBUG_LOG("[BATTERY]", "Warning: INA219 overflow detected!");
        return false;
    }

    return true;
}

float battery_get_percentage(float voltage) {
    // Clamp voltage to valid range
    if (voltage >= Battery::VOLTAGE_MAX) {
        return 1.0f;  // 100%
    }
    if (voltage <= Battery::VOLTAGE_MIN) {
        return 0.0f;  // 0%
    }

    // Linear approximation of 3S LiPo discharge curve
    // More accurate would use a lookup table, but this is sufficient
    float percentage = (voltage - Battery::VOLTAGE_MIN) / Battery::VOLTAGE_RANGE;

    return constrain(percentage, 0.0f, 1.0f);
}

bool is_battery_ready() {
    return battery_initialized;
}
