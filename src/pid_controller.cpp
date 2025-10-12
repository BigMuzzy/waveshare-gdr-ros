/**
 * @file pid_controller.cpp
 * @brief PID speed control implementation
 *
 * Implements feedforward + PID control for differential drive motors.
 * Based on reference/general_driver/movtion_module.h::pidControllerInit()
 */

#include "pid_controller.h"
#include "debug_serial.h"
#include <Arduino.h>

// Internal variables for PID computation (must match PID library naming)
static double left_input = 0.0;
static double left_output = 0.0;
static double left_setpoint = 0.0;

static double right_input = 0.0;
static double right_output = 0.0;
static double right_setpoint = 0.0;

// PID controller objects (instantiated here, declared extern in header)
// Constructor: PID(Input*, Output*, Setpoint*, Kp, Ki, Kd, Direction)
// Note: PID library object names kept as pidLeft/pidRight (external library convention)
PID pidLeft(&left_input, &left_output, &left_setpoint, PIDConfig::KP, PIDConfig::KI, PIDConfig::KD, DIRECT);
PID pidRight(&right_input, &right_output, &right_setpoint, PIDConfig::KP, PIDConfig::KI, PIDConfig::KD, DIRECT);

void pid_init() {
    // Set output limits
    pidLeft.SetOutputLimits(PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);
    pidRight.SetOutputLimits(PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);

    // Turn on automatic mode
    pidLeft.SetMode(AUTOMATIC);
    pidRight.SetMode(AUTOMATIC);

    debug_log("PID", "Controllers initialized");
    debug_printf("[PID] Kp=%.1f, Ki=%.1f, Kd=%.1f\n", PIDConfig::KP, PIDConfig::KI, PIDConfig::KD);
    debug_printf("[PID] Output: [%.0f, %.0f], Threshold: %d\n",
                 PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX, PIDConfig::THRESHOLD);
}

void pid_compute(float target_left_speed, float target_right_speed,
                 float measured_left_speed, float measured_right_speed,
                 int16_t &pwm_left_output, int16_t &pwm_right_output) {

    // --- FEEDFORWARD: Direct speed → PWM mapping ---
    // Motor needs ~255 PWM for 0.463 m/s max speed
    const float FEEDFORWARD_GAIN = 255.0 / 0.463;  // ≈ 550 PWM per m/s

    float ff_left = target_left_speed * FEEDFORWARD_GAIN;
    float ff_right = target_right_speed * FEEDFORWARD_GAIN;

    // --- PID: Correct errors ---
    left_setpoint = target_left_speed;
    right_setpoint = target_right_speed;
    left_input = measured_left_speed;
    right_input = measured_right_speed;

    pidLeft.Compute();
    pidRight.Compute();

    // --- COMBINE: Feedforward + PID correction ---
    double final_left = ff_left + left_output;
    double final_right = ff_right + right_output;

    // Apply threshold and limits
    if (abs(final_left) < PIDConfig::THRESHOLD) final_left = 0.0;
    if (abs(final_right) < PIDConfig::THRESHOLD) final_right = 0.0;

    if (left_setpoint == 0.0 && measured_left_speed == 0.0) final_left = 0.0;
    if (right_setpoint == 0.0 && measured_right_speed == 0.0) final_right = 0.0;

    pwm_left_output = (int16_t)constrain(final_left, PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);
    pwm_right_output = (int16_t)constrain(final_right, PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);

    // debug_printf("right pwm %i | target: %.3f | measured: %.3f\n",
    //              pwm_right_output, target_right_speed, measured_right_speed);
}


void pid_reset() {
    // Turn off controllers
    pidLeft.SetMode(MANUAL);
    pidRight.SetMode(MANUAL);

    // Clear internal states
    left_input = 0.0;
    left_output = 0.0;
    left_setpoint = 0.0;
    right_input = 0.0;
    right_output = 0.0;
    right_setpoint = 0.0;

    // Turn controllers back on (this clears integral term)
    pidLeft.SetMode(AUTOMATIC);
    pidRight.SetMode(AUTOMATIC);
}

void set_pid_tunings(float kp, float ki, float kd) {
    pidLeft.SetTunings(kp, ki, kd);
    pidRight.SetTunings(kp, ki, kd);
}
