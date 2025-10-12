# Coding Style Guide - waveshare-gdr-ros

## Overview
This document defines the coding standards for the waveshare-gdr-ros project. Consistent style improves readability, reduces errors, and makes the codebase easier to maintain.

**Version:** 1.0
**Last Updated:** 2025-10-11

---

## Quick Reference Table

| Element | Convention | Example | Notes |
|---------|-----------|---------|-------|
| **Functions** | `snake_case` | `motor_init()`, `set_motor_speed()` | C-compatible, readable |
| **Variables** | `snake_case` | `target_speed`, `encoder_count` | Local and global |
| **Constants** | `UPPER_SNAKE_CASE` | `Motor::PWM_FREQUENCY` | With namespace, constexpr |
| **Types/Classes** | `PascalCase` | `MotorController`, `EncoderData` | Clear distinction from variables |
| **Enums** | `PascalCase` | `enum class State` | Modern C++ style |
| **Enum Values** | `UPPER_SNAKE_CASE` | `State::RUNNING` | Clear, consistent |
| **Namespaces** | `PascalCase` | `Motor`, `Encoder`, `PID` | Clear, modular organization |
| **Macros** | `UPPER_SNAKE_CASE` | `DEBUG_ENABLED` | Minimize use, prefer constexpr |
| **Files** | `snake_case` | `motor_control.cpp` | Lowercase with underscores |

---

## Detailed Naming Conventions

### 1. Functions: `snake_case`

**Apply to:** All function names, including member functions

```cpp
// ✅ Correct
void motor_init();
void set_motor_speed(int16_t left, int16_t right);
void get_wheel_speeds(float& left, float& right);
void pid_compute(float setpoint, float measured, float& output);
bool is_motor_enabled();
float calculate_distance(int32_t pulses);

// ❌ Incorrect (old mixed style)
void motorInit();           // camelCase
void setMotorSpeed();       // camelCase
void getWheelSpeeds();      // camelCase
```

**Rationale:**
- C-compatible naming
- Excellent readability in embedded context
- Common in safety-critical and embedded systems
- Works well on small displays/terminals
- Separates words clearly without case sensitivity

**Special Exception:** Keep ROS/micro-ROS callback and interface functions in their original style to match ROS conventions:
```cpp
// ✅ Keep these as-is (ROS convention)
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void cmd_vel_callback(const void* msgin);
void create_entities();
void destroy_entities();
void init_ros_msgs();
```

---

### 2. Variables: `snake_case`

**Apply to:** All variables (local, global, static, member variables)

```cpp
// ✅ Correct - Local variables
float target_speed = 0.0f;
int32_t encoder_count = 0;
bool motor_enabled = false;
uint8_t buffer_index = 0;

// ✅ Correct - Global/file-static variables (optional g_ prefix)
static float g_target_left_speed = 0.0f;    // Option 1: prefix with g_
static float target_right_speed = 0.0f;     // Option 2: no prefix (acceptable)
static bool g_initialized = false;

// ✅ Correct - Member variables (optional m_ prefix)
class MotorController {
    float m_current_speed;     // Option 1: prefix with m_
    int16_t pwm_output;        // Option 2: no prefix (acceptable)
    bool m_is_running;
};

// ❌ Incorrect (old mixed style)
float leftSpeed;              // camelCase
double setpointA;             // camelCase with abbreviation
int encoderPulsesB;           // camelCase
float speedGetA;              // camelCase
```

**Rationale:**
- Consistency with function naming
- Easy to read and type
- No ambiguity about word boundaries
- Works well with code completion

**Special Exception:** ROS message data can keep original naming to match ROS conventions:
```cpp
// ✅ Keep these as-is (matches ROS msg fields)
float cmd_linear_x;
float cmd_angular_z;
bool state_connected;
bool motor_enabled;
```

---

### 3. Constants: `UPPER_SNAKE_CASE` with `constexpr`

**Apply to:** All compile-time constants

```cpp
// ✅ Correct - Namespace + constexpr + UPPER_SNAKE_CASE
namespace Motor {
    constexpr uint8_t PWM_PIN_LEFT = 25;
    constexpr uint8_t PWM_PIN_RIGHT = 26;
    constexpr uint32_t PWM_FREQUENCY = 100000;
    constexpr int16_t PWM_MAX = 255;
    constexpr int16_t PWM_MIN = -255;
    constexpr int16_t THRESHOLD_PWM = 23;
}

namespace PID {
    constexpr double KP = 20.0;
    constexpr double KI = 2000.0;
    constexpr double KD = 0.0;
    constexpr double OUTPUT_MIN = -255.0;
    constexpr double OUTPUT_MAX = 255.0;
}

namespace Robot {
    constexpr double WHEEL_DIAMETER_M = 0.068;
    constexpr double WHEEL_BASE_M = 0.200;
}

// ✅ Acceptable - For single-file constants
constexpr uint32_t BUFFER_SIZE = 256;
constexpr float GRAVITY_MPS2 = 9.80665;

// ❌ Incorrect - #define without namespace (old style)
#define PWMA 25                    // Replace with namespace constant
#define PWM_FREQ 100000           // Replace with Motor::PWM_FREQUENCY
#define MAX_SPEED 2.0             // Replace with Robot::MAX_SPEED

// ❌ Incorrect - camelCase constants
constexpr float maxSpeed = 2.0f;   // Use MAX_SPEED
constexpr int bufferSize = 256;    // Use BUFFER_SIZE
```

**Rationale:**
- Clear visual distinction from variables
- Matches industry standard for constants
- Namespace organization prevents name collisions
- `constexpr` provides type safety and compiler optimizations
- Traditional uppercase constant style

---

### 4. Types and Classes: `PascalCase`

**Apply to:** Class names, struct names, typedef names, type aliases

```cpp
// ✅ Correct - Classes
class MotorController {
public:
    void update();
    float get_speed() const;
};

class PidController {
    double compute(double setpoint, double measured);
};

// ✅ Correct - Structs
struct WheelSpeeds {
    float left;
    float right;
};

struct EncoderData {
    int32_t count;
    float speed;
    uint32_t timestamp;
};

struct RobotPose {
    double x;
    double y;
    double theta;
};

// ✅ Correct - Type aliases
using SpeedArray = std::array<float, 2>;
using SpeedCallback = std::function<void(float)>;
using EncoderCallback = void(*)(int32_t);

// ❌ Incorrect
class motor_controller { };        // snake_case - use PascalCase
struct wheel_speeds { };           // snake_case - use PascalCase
class MOTOR_CTRL { };              // UPPER_SNAKE - use PascalCase
```

**Rationale:**
- Clear distinction between types and variables
- Standard C++ convention
- Easy to identify type names in code
- Matches STL naming for user-defined types

---

### 5. Enums: `PascalCase` for type, `UPPER_SNAKE_CASE` for values

```cpp
// ✅ Correct - Modern C++ enum class
enum class State {
    IDLE,
    RUNNING,
    ERROR,
    FAULT
};

enum class MotorDirection {
    FORWARD,
    REVERSE,
    STOPPED
};

enum class ConnectionState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

// Usage
State current_state = State::IDLE;
MotorDirection dir = MotorDirection::FORWARD;

// ✅ Acceptable - C-style enum (legacy code, ROS compatibility)
enum ConnectionState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

// ❌ Incorrect - camelCase values
enum class State {
    Idle,      // Use IDLE
    Running,   // Use RUNNING
    Error      // Use ERROR
};

// ❌ Incorrect - snake_case type
enum class motor_state {  // Use MotorState
    IDLE,
    RUNNING
};
```

**Rationale:**
- `enum class` provides type safety
- PascalCase type matches other types
- UPPER_SNAKE values match constant convention
- Clear distinction between enum type and values

---

### 6. Namespaces: `PascalCase`

```cpp
// ✅ Correct
namespace Motor {
    constexpr uint8_t PWM_PIN = 25;
    void init();
}

namespace Encoder {
    void init();
    void reset();
}

namespace PID {
    constexpr double KP = 20.0;
}

namespace Robot {
    constexpr double WHEEL_BASE_M = 0.200;
}

// ✅ Correct - Acronyms stay uppercase
namespace I2C {
    constexpr uint8_t SDA_PIN = 32;
}

namespace ROS {
    constexpr const char* NODE_NAME = "general_driver";
}

namespace PWM {
    constexpr uint32_t FREQUENCY = 100000;
}

// ❌ Incorrect
namespace motor { }           // lowercase - use Motor
namespace motor_control { }   // snake_case - use MotorControl
namespace MOTOR { }           // UPPER_SNAKE - use Motor
```

**Rationale:**
- Matches class/type naming convention
- Clear, professional appearance
- Distinguishes namespaces from functions
- Industry standard for C++ namespaces

---

### 7. File Names: `snake_case.cpp` / `snake_case.h`

```bash
# ✅ Correct
motor_control.cpp / motor_control.h
pid_controller.cpp / pid_controller.h
encoder_handler.cpp / encoder_handler.h
kinematics.cpp / kinematics.h
debug_serial.cpp / debug_serial.h
ros_com.cpp / ros_com.h
battery_monitor.cpp / battery_monitor.h
imu_handler.cpp / imu_handler.h

# ❌ Incorrect
motorControl.cpp          // camelCase
MotorControl.cpp         // PascalCase
motor-control.cpp        // kebab-case
Motor_Control.cpp        // Mixed case
```

**Rationale:**
- Consistent with Linux/Unix conventions
- Avoids case-sensitivity issues across platforms
- Matches function naming style
- Clear word separation

---

## Code Organization

### Header Guards

**Preferred:** Use `#pragma once` for simplicity (widely supported by modern compilers)

```cpp
#pragma once

// Header content
```

**Alternative:** Traditional include guards (acceptable for maximum compatibility)

```cpp
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Header content

#endif // MOTOR_CONTROL_H
```

**Rationale:**
- `#pragma once` is simpler and less error-prone
- Supported by GCC, Clang, MSVC, and embedded toolchains
- No risk of naming collisions in guard names

---

### Include Order

Organize includes in this order for clarity and to catch missing dependencies:

```cpp
// In motor_control.cpp

// 1. Corresponding header (for .cpp files)
#include "motor_control.h"

// 2. C system headers
#include <stdint.h>
#include <string.h>
#include <math.h>

// 3. C++ standard library headers
#include <vector>
#include <array>
#include <functional>

// 4. Third-party library headers
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <ESP32Encoder.h>

// 5. Project headers
#include "config.h"
#include "debug_serial.h"
#include "encoder_handler.h"
```

**Rationale:**
- Own header first catches missing dependencies in header
- Logical grouping improves readability
- Standard convention in many codebases

---

### Namespace Usage

**In Header Files:**
```cpp
// ✅ Define namespaces, never use "using"
namespace Motor {
    constexpr uint8_t PWM_PIN = 25;
    void init();
    void set_speed(float speed);
}

// ❌ NEVER in headers
using namespace std;   // Pollutes global namespace for all includers
using namespace Motor; // Forces names on all includers
```

**In Implementation Files:**
```cpp
// ✅ Using specific symbols is OK
using Motor::PWM_PIN;
using Motor::PWM_FREQUENCY;

// ✅ Using namespace in .cpp is acceptable
using namespace Motor;

// ✅ Acceptable for standard library
using std::vector;
using std::array;

// ⚠️ Use sparingly (can cause confusion)
using namespace std;   // Prefer specific using declarations
```

**Rationale:**
- Prevents namespace pollution
- Headers should not force names into includer's scope
- Implementation files can use shortcuts for convenience

---

## Formatting

### Indentation
- **4 spaces** (no tabs)
- Consistent indentation for all blocks
- Configure your editor to insert spaces

```cpp
void function_name()
{
    if (condition) {
        do_something();
        if (nested) {
            do_more();
        }
    }
}
```

---

### Braces

**Option 1: Allman Style** (recommended for embedded/safety-critical)
```cpp
void function_name()
{
    if (condition)
    {
        do_something();
    }
    else
    {
        do_other();
    }
}
```

**Option 2: K&R Style** (acceptable alternative)
```cpp
void function_name() {
    if (condition) {
        do_something();
    } else {
        do_other();
    }
}
```

**Important:** Be consistent within each file. Don't mix styles.

---

### Line Length
- Prefer **100 characters maximum**
- Break long lines at logical points
- Indent continuation lines for clarity

```cpp
// ✅ Good - broken at logical point
void set_motor_parameters(
    float speed,
    MotorDirection direction,
    bool enable_pid
);

// ✅ Good - aligned parameters
float result = calculate_odometry(
    encoder_left, encoder_right,
    wheel_base, wheel_diameter
);
```

---

### Spacing

```cpp
// ✅ Spaces around binary operators
int result = a + b * c;
float ratio = distance / time;
bool valid = (x == 0) && (y > 10);

// ✅ Space after keywords
if (condition)
for (int i = 0; i < 10; i++)
while (running)
switch (state)

// ✅ No space between function name and parenthesis
void function_name(int param);
my_function(arg1, arg2);
calculate_speed(encoder_pulses);

// ✅ Space after comma in parameter lists
void func(int a, int b, int c);

// ❌ Avoid
int result=a+b*c;           // No spaces
if( condition ) { }         // Extra spaces inside parens
void function_name (int);   // Space before (
void func(int a,int b);     // No space after comma
```

---

## Comments

### File Headers

Every source file should have a header comment:

```cpp
/**
 * @file motor_control.cpp
 * @brief Motor PWM control and direction management
 *
 * Implements H-bridge motor control using ESP32 LEDC peripheral.
 * Supports bidirectional control with PWM speed regulation.
 *
 * Hardware:
 * - Left Motor: PWM=GPIO25, DIR1=GPIO21, DIR2=GPIO17
 * - Right Motor: PWM=GPIO26, DIR1=GPIO22, DIR2=GPIO23
 *
 * @author Your Name
 * @date 2025-10-11
 */
```

---

### Function Documentation

Use Doxygen-style comments for public functions:

```cpp
/**
 * @brief Initialize motor control hardware
 *
 * Configures PWM channels and direction control pins for
 * left and right motors. Must be called before any motor
 * control operations.
 *
 * @note PWM frequency is set to 100kHz with 8-bit resolution
 */
void motor_init();

/**
 * @brief Set motor speeds with PWM control
 *
 * @param left_pwm Left motor PWM value (-255 to +255)
 * @param right_pwm Right motor PWM value (-255 to +255)
 *
 * Positive values drive motors forward, negative values reverse.
 * Values below threshold (23) are treated as zero to prevent stalling.
 *
 * @warning Ensure motor_init() has been called first
 */
void set_motor_speed(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Calculate distance traveled from encoder pulses
 *
 * @param pulses Number of encoder pulses
 * @return Distance in meters
 *
 * Uses wheel diameter and encoder PPR from Robot namespace.
 */
float calculate_distance(int32_t pulses);
```

---

### Inline Comments

- Explain **why**, not **what** (code should be self-explanatory)
- Use `//` for single-line comments
- Keep comments up-to-date with code changes
- Delete commented-out code (use version control instead)

```cpp
// ✅ Good - explains reasoning
if (speed < Motor::THRESHOLD_PWM) {
    // Below threshold, motor won't move effectively due to friction
    // Better to stop completely than apply insufficient torque
    speed = 0;
}

// ✅ Good - explains non-obvious logic
// Convert theta to quaternion using half-angle formula
// This avoids gimbal lock for full 360° rotation
odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0);
odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0);

// ✅ Good - warns about important constraints
// CRITICAL: Must call this before micro-ROS agent connection
// Serial port is shared with micro-ROS transport
set_microros_serial_transports(Serial);

// ❌ Bad - states the obvious
if (speed < Motor::THRESHOLD_PWM) {
    // Set speed to zero
    speed = 0;
}

// ❌ Bad - commented-out code (delete it!)
// float old_speed = get_old_speed();
// calculate_something(old_speed);
```

---

## Best Practices

### Use `constexpr` Instead of `#define`

```cpp
// ✅ Preferred - Type-safe, scoped, debuggable
namespace Motor {
    constexpr uint8_t PWM_PIN = 25;
    constexpr uint32_t PWM_FREQUENCY = 100000;
}

// Usage
ledcAttachPin(Motor::PWM_PIN, channel);

// ❌ Avoid - No type safety, global scope
#define PWM_PIN 25
#define PWM_FREQUENCY 100000
```

**Exception:** Use `#define` for:
- Include guards (or use `#pragma once`)
- Conditional compilation (`#ifdef`, `#ifndef`)
- Arduino/platform-specific macros

---

### Use `nullptr` Instead of `NULL`

```cpp
// ✅ C++11 and later
void* ptr = nullptr;
rcl_timer_t* timer = nullptr;

if (timer != nullptr) {
    // ...
}

// ❌ C-style (avoid in C++ code)
void* ptr = NULL;
void* ptr = 0;
```

---

### Use `enum class` Instead of `enum`

```cpp
// ✅ Type-safe, scoped
enum class State {
    IDLE,
    RUNNING,
    ERROR
};

State current_state = State::IDLE;

// Prevents accidental comparisons between different enums
// State s1 = State::IDLE;
// MotorDirection d1 = MotorDirection::FORWARD;
// if (s1 == d1) { } // Compile error - good!

// ❌ Pollutes namespace, not type-safe
enum State {
    IDLE,      // Now IDLE is global symbol
    RUNNING,
    ERROR
};

State s = IDLE;  // Name collision risk
```

**Exception:** Use C-style `enum` when required for:
- ROS compatibility
- Third-party library interfaces
- C compatibility requirements

---

### Initialize Variables at Declaration

```cpp
// ✅ Initialize when declaring
float speed = 0.0f;
bool enabled = false;
int32_t count = 0;
State state = State::IDLE;

// ✅ Use {} initialization for clarity
float speeds[2] = {0.0f, 0.0f};
std::array<float, 2> speeds{0.0f, 0.0f};

// ❌ Uninitialized (undefined behavior)
float speed;           // Contains garbage
bool enabled;          // Unpredictable value
int32_t count;         // Random value
```

---

### Use `const` and `constexpr` Liberally

```cpp
// ✅ Mark compile-time constants as constexpr
constexpr float PI = 3.14159265359f;
constexpr int BUFFER_SIZE = 256;

// ✅ Mark runtime constants as const
const float measured_speed = get_current_speed();
const int32_t encoder_count = read_encoder();

// ✅ Const parameters prevent accidental modification
void process_data(const float* data, size_t length);
void calculate_odometry(const int32_t delta_left, const int32_t delta_right);

// ✅ Const member functions (don't modify object state)
class Motor {
public:
    float get_speed() const;      // Doesn't modify member variables
    bool is_enabled() const;
};
```

---

## Embedded-Specific Guidelines

### Memory Management

**Avoid dynamic allocation in real-time code:**

```cpp
// ✅ Static allocation (preferred for embedded)
static uint8_t buffer[256];
static float speed_history[100];

// ✅ Stack allocation (OK for small, temporary buffers)
void function() {
    uint8_t temp_buffer[64];     // Small buffer on stack
    float calculations[10];      // Temporary array
}

// ✅ Pre-allocate at startup
void setup() {
    static std::array<EncoderData, 100> encoder_history;
    // Use throughout program lifetime
}

// ❌ Avoid in embedded real-time code
void control_loop() {
    uint8_t* buffer = new uint8_t[1024];  // Heap allocation - risky!
    std::vector<float> data;              // Dynamic growth - avoid in ISR/RT
    delete[] buffer;
}

// ⚠️ Acceptable only at startup/initialization
void setup() {
    std::vector<Config> configs;  // OK if allocated once at startup
    configs.reserve(10);          // Pre-allocate if size is known
}
```

**Rationale:**
- Prevents heap fragmentation
- Deterministic memory usage
- Faster allocation/deallocation
- No risk of malloc failure in critical code

---

### Interrupt Safety

**Keep ISRs short and fast:**

```cpp
// ✅ Good - Minimal work in ISR
volatile bool data_ready = false;
volatile int32_t encoder_count = 0;

void IRAM_ATTR encoder_isr() {
    encoder_count++;          // Quick increment
    data_ready = true;        // Set flag
}

void loop() {
    if (data_ready) {
        process_encoder_data(encoder_count);  // Heavy work in main loop
        data_ready = false;
    }
}

// ✅ Use volatile for ISR-modified variables
volatile uint32_t interrupt_count = 0;
volatile bool flag = false;

// ❌ Bad - Too much work in ISR
void IRAM_ATTR bad_isr() {
    float speed = calculate_speed();     // Complex calculation
    update_display(speed);               // I/O operation
    log_to_serial(speed);                // Serial communication
}
```

**Rationale:**
- ISRs must execute quickly
- Long ISRs block other interrupts
- Use flags to defer work to main loop
- `volatile` ensures compiler doesn't optimize away shared variables

---

### Resource Management

**Check return values and handle errors:**

```cpp
// ✅ Good - Check initialization results
bool motor_init() {
    if (!ledcSetup(channel, freq, resolution)) {
        debug_log("ERROR", "PWM setup failed");
        return false;
    }
    return true;
}

// ✅ Good - Validate inputs
void set_motor_speed(int16_t pwm) {
    if (pwm < Motor::PWM_MIN || pwm > Motor::PWM_MAX) {
        debug_log("WARN", "PWM out of range, clamping");
        pwm = constrain(pwm, Motor::PWM_MIN, Motor::PWM_MAX);
    }
    apply_pwm(pwm);
}

// ✅ Good - Clean up in reverse order of acquisition
void shutdown() {
    stop_motors();        // Stop using hardware
    disable_pwm();        // Disable PWM
    deinit_gpio();        // Release GPIO
}

// ❌ Bad - Ignoring return values
void setup() {
    ledcSetup(channel, freq, resolution);  // What if it fails?
    Wire.begin();                          // Check if I2C initialized?
}
```

---

## Complete Examples

### Example: Header File

```cpp
#pragma once

#include <stdint.h>

/**
 * @file motor_control.h
 * @brief Motor control interface for differential drive robot
 */

namespace Motor {
    // Hardware pin definitions
    constexpr uint8_t PWM_PIN_LEFT = 25;
    constexpr uint8_t PWM_PIN_RIGHT = 26;
    constexpr uint8_t DIR1_PIN_LEFT = 21;
    constexpr uint8_t DIR2_PIN_LEFT = 17;
    constexpr uint8_t DIR1_PIN_RIGHT = 22;
    constexpr uint8_t DIR2_PIN_RIGHT = 23;

    // PWM configuration
    constexpr uint32_t PWM_FREQUENCY = 100000;
    constexpr uint8_t PWM_RESOLUTION_BITS = 8;
    constexpr int16_t PWM_MAX = 255;
    constexpr int16_t PWM_MIN = -255;
    constexpr int16_t THRESHOLD_PWM = 23;
}

/**
 * @brief Initialize motor control subsystem
 * @return true if initialization successful, false otherwise
 */
bool motor_init();

/**
 * @brief Set motor speeds with PWM control
 * @param left_pwm Left motor PWM value (-255 to +255)
 * @param right_pwm Right motor PWM value (-255 to +255)
 */
void set_motor_speed(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Emergency stop all motors
 */
void emergency_stop();

/**
 * @brief Check if motors are initialized
 * @return true if motors ready, false otherwise
 */
bool is_motor_ready();
```

---

### Example: Implementation File

```cpp
#include "motor_control.h"
#include "config.h"
#include "debug_serial.h"

#include <Arduino.h>

// File-scope static variables
static bool g_motors_initialized = false;

bool motor_init()
{
    debug_log("MOTOR", "Initializing motor control...");

    // Configure left motor PWM
    if (!ledcSetup(BSP::MotorDriver::PWM_CHANNEL_A,
                   Motor::PWM_FREQUENCY,
                   Motor::PWM_RESOLUTION_BITS)) {
        debug_log("ERROR", "Left PWM setup failed");
        return false;
    }
    ledcAttachPin(Motor::PWM_PIN_LEFT, BSP::MotorDriver::PWM_CHANNEL_A);

    // Configure direction pins
    pinMode(Motor::DIR1_PIN_LEFT, OUTPUT);
    pinMode(Motor::DIR2_PIN_LEFT, OUTPUT);
    pinMode(Motor::DIR1_PIN_RIGHT, OUTPUT);
    pinMode(Motor::DIR2_PIN_RIGHT, OUTPUT);

    // Ensure motors start stopped
    emergency_stop();

    g_motors_initialized = true;
    debug_log("MOTOR", "Initialization complete");

    return true;
}

void set_motor_speed(int16_t left_pwm, int16_t right_pwm)
{
    // Validate initialization
    if (!g_motors_initialized) {
        debug_log("ERROR", "Motors not initialized");
        return;
    }

    // Clamp to valid range
    left_pwm = constrain(left_pwm, Motor::PWM_MIN, Motor::PWM_MAX);
    right_pwm = constrain(right_pwm, Motor::PWM_MIN, Motor::PWM_MAX);

    // Apply threshold - below this, motor won't move effectively
    if (abs(left_pwm) < Motor::THRESHOLD_PWM) {
        left_pwm = 0;
    }
    if (abs(right_pwm) < Motor::THRESHOLD_PWM) {
        right_pwm = 0;
    }

    // Set left motor direction and PWM
    if (left_pwm > 0) {
        digitalWrite(Motor::DIR1_PIN_LEFT, HIGH);
        digitalWrite(Motor::DIR2_PIN_LEFT, LOW);
    } else if (left_pwm < 0) {
        digitalWrite(Motor::DIR1_PIN_LEFT, LOW);
        digitalWrite(Motor::DIR2_PIN_LEFT, HIGH);
    } else {
        digitalWrite(Motor::DIR1_PIN_LEFT, LOW);
        digitalWrite(Motor::DIR2_PIN_LEFT, LOW);
    }

    ledcWrite(BSP::MotorDriver::PWM_CHANNEL_A, abs(left_pwm));

    // (Repeat for right motor...)
}

void emergency_stop()
{
    if (!g_motors_initialized) {
        return;
    }

    // Set all direction pins LOW (brake mode)
    digitalWrite(Motor::DIR1_PIN_LEFT, LOW);
    digitalWrite(Motor::DIR2_PIN_LEFT, LOW);
    digitalWrite(Motor::DIR1_PIN_RIGHT, LOW);
    digitalWrite(Motor::DIR2_PIN_RIGHT, LOW);

    // Set PWM to zero
    ledcWrite(BSP::MotorDriver::PWM_CHANNEL_A, 0);
    ledcWrite(BSP::MotorDriver::PWM_CHANNEL_B, 0);

    debug_log("MOTOR", "Emergency stop activated");
}

bool is_motor_ready()
{
    return g_motors_initialized;
}
```

---

## Migration Guidelines

When refactoring existing code to match this style guide:

### 1. **One Module at a Time**
- Refactor related .cpp/.h files together
- Don't mix old and new styles within a file
- Test after each module is complete

### 2. **Update Systematically**
```bash
# Find all camelCase function calls
grep -r "motorInit\|setMotorSpeed\|encoderInit" src/ include/

# Use editor find-replace or sed for batch updates
sed -i 's/motorInit/motor_init/g' src/*.cpp include/*.h
```

### 3. **Maintain Functionality**
- Refactoring should NOT change behavior
- Keep the same logic, just rename identifiers
- Verify with compilation and testing

### 4. **Preserve ROS Conventions**
Keep ROS-related names unchanged:
- `timer_callback()`, `cmd_vel_callback()`
- `create_entities()`, `destroy_entities()`
- `init_ros_msgs()`, `ros_loop()`

### 5. **Document Changes**
```bash
git commit -m "refactor: Rename motorInit() to motor_init() for consistency

- Apply snake_case naming convention to motor control functions
- Update all call sites in main.cpp and ros_com.cpp
- No functional changes, style update only"
```

---

## Questions and Clarifications

For questions about this style guide:
- Create an issue in the project repository
- Discuss with the team before making style changes
- Propose amendments via pull request

---

## Acknowledgments

This style guide is influenced by:
- Google C++ Style Guide
- Linux Kernel Coding Style
- MISRA C Guidelines
- ROS2 Coding Standards
- Embedded C++ best practices

---

**Last Updated:** 2025-10-11
**Version:** 1.0
**Maintainer:** waveshare-gdr-ros project team
