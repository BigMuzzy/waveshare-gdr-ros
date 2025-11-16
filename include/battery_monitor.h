/**
 * @file battery_monitor.h
 * @brief Battery voltage and current monitoring using INA219
 *
 * Monitors 3S LiPo battery (9.0V - 12.6V) via INA219 power monitor.
 * Based on reference/general_driver/General_Driver/battery_ctrl.h
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

/**
 * @brief Initialize INA219 battery monitor
 * @return true if initialization successful, false otherwise
 *
 * Configures INA219 with:
 * - 16V bus range (for 3S LiPo)
 * - ±320mV shunt voltage range
 * - 9-bit ADC resolution
 * - 0.01Ω shunt resistor
 */
bool battery_init();

/**
 * @brief Read current battery measurements
 * @param voltage Output: battery voltage in volts
 * @param current Output: battery current in amperes
 * @param power Output: battery power in watts
 * @return true if read successful, false otherwise
 *
 * Updates all battery measurements from INA219.
 * Call this before publishing battery data.
 */
bool battery_read(float &voltage, float &current, float &power);

/**
 * @brief Calculate battery percentage from voltage
 * @param voltage Battery voltage in volts
 * @return Battery percentage (0.0 to 1.0)
 *
 * Uses 3S LiPo discharge curve:
 * - 12.6V = 100% (4.2V per cell)
 * - 11.1V = ~50% (3.7V per cell nominal)
 * - 9.0V  = 0%   (3.0V per cell minimum)
 */
float battery_get_percentage(float voltage);

/**
 * @brief Check if battery monitor is ready
 * @return true if INA219 initialized successfully
 */
bool is_battery_ready();

#endif // BATTERY_MONITOR_H
