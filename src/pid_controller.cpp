#include "pid_controller.h"
#include "debug_serial.h"
#include <Arduino.h>

// Internal variables for PID computation
static double leftInput = 0.0;
static double leftOutput = 0.0;
static double leftSetpoint = 0.0;

static double rightInput = 0.0;
static double rightOutput = 0.0;
static double rightSetpoint = 0.0;

// PID controller objects (instantiated here, declared extern in header)
// Constructor: PID(Input*, Output*, Setpoint*, Kp, Ki, Kd, Direction)
PID pidLeft(&leftInput, &leftOutput, &leftSetpoint, PIDConfig::KP, PIDConfig::KI, PIDConfig::KD, DIRECT);
PID pidRight(&rightInput, &rightOutput, &rightSetpoint, PIDConfig::KP, PIDConfig::KI, PIDConfig::KD, DIRECT);

/**
 * Initialize PID controllers
 * Based on reference/general_driver/movtion_module.h::pidControllerInit()
 */
void pidInit() {
    // Set output limits
    pidLeft.SetOutputLimits(PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);
    pidRight.SetOutputLimits(PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);

    // Turn on automatic mode
    pidLeft.SetMode(AUTOMATIC);
    pidRight.SetMode(AUTOMATIC);

    debugLog("PID", "Controllers initialized");
    debugPrintf("[PID] Kp=%.1f, Ki=%.1f, Kd=%.1f\n", PIDConfig::KP, PIDConfig::KI, PIDConfig::KD);
    debugPrintf("[PID] Output: [%.0f, %.0f], Threshold: %d\n",
                PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX, PIDConfig::THRESHOLD);
}

/**
 * Compute PID outputs for both wheels
 * Based on reference/general_driver/movtion_module.h::pidControllerCompute()
 *
 * @param left_setpoint  Target speed for left wheel (m/s)
 * @param right_setpoint Target speed for right wheel (m/s)
 * @param left_speed     Actual speed of left wheel (m/s)
 * @param right_speed    Actual speed of right wheel (m/s)
 * @param left_output    Reference to store left motor PWM output
 * @param right_output   Reference to store right motor PWM output
 */
void pidCompute(float left_setpoint, float right_setpoint,
                float left_speed, float right_speed,
                int16_t &left_output, int16_t &right_output) {

    // --- FEEDFORWARD: Direct speed → PWM mapping ---
    // Your motor needs ~255 PWM for 0.463 m/s max speed
    const float FEEDFORWARD_GAIN = 255.0 / 0.463;  // ≈ 550 PWM per m/s
    
    float ff_left = left_setpoint * FEEDFORWARD_GAIN;
    float ff_right = right_setpoint * FEEDFORWARD_GAIN;
    
    // --- PID: Correct errors ---
    leftSetpoint = left_setpoint;
    rightSetpoint = right_setpoint;
    leftInput = left_speed;
    rightInput = right_speed;
    
    pidLeft.Compute();
    pidRight.Compute();
    
    // --- COMBINE: Feedforward + PID correction ---
    double finalLeftOutput = ff_left + leftOutput;
    double finalRightOutput = ff_right + rightOutput;
    
    // Apply threshold and limits
    if (abs(finalLeftOutput) < PIDConfig::THRESHOLD) finalLeftOutput = 0.0;
    if (abs(finalRightOutput) < PIDConfig::THRESHOLD) finalRightOutput = 0.0;
    
    if (leftSetpoint == 0.0 && left_speed == 0.0) finalLeftOutput = 0.0;
    if (rightSetpoint == 0.0 && right_speed == 0.0) finalRightOutput = 0.0;
    
    left_output = (int16_t)constrain(finalLeftOutput, PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);
    right_output = (int16_t)constrain(finalRightOutput, PIDConfig::OUTPUT_MIN, PIDConfig::OUTPUT_MAX);

    debugPrintf("right pwm %i | rsp: %f | rcs %f\n", right_output, right_setpoint, right_speed);
}


/**
 * Reset PID controllers (clear integral windup)
 */
void pidReset() {
    // Turn off controllers
    pidLeft.SetMode(MANUAL);
    pidRight.SetMode(MANUAL);

    // Clear internal states
    leftInput = 0.0;
    leftOutput = 0.0;
    leftSetpoint = 0.0;
    rightInput = 0.0;
    rightOutput = 0.0;
    rightSetpoint = 0.0;

    // Turn controllers back on (this clears integral term)
    pidLeft.SetMode(AUTOMATIC);
    pidRight.SetMode(AUTOMATIC);
}

/**
 * Update PID tuning parameters
 * Based on reference/general_driver/movtion_module.h::setPID()
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void setPIDTunings(float kp, float ki, float kd) {
    pidLeft.SetTunings(kp, ki, kd);
    pidRight.SetTunings(kp, ki, kd);
}
