# Refactoring Checklist - waveshare-gdr-ros

## Status: IN PROGRESS

### ✅ Completed Modules

- [x] **docs/CODING_STYLE.md** - Comprehensive style guide created
- [x] **motor_control.cpp/h** - Functions and documentation updated
- [x] **debug_serial.cpp/h** - Functions and documentation updated
- [x] **pid_controller.cpp/h** - Functions, variables, and documentation updated

---

### 🔄 Remaining Modules

#### 1. encoder_handler.cpp/h

**Functions to rename:**
```cpp
// OLD → NEW
encoderInit() → encoder_init()
getEncoderCounts() → get_encoder_counts()
getWheelSpeeds() → get_wheel_speeds()
resetEncoders() → reset_encoders()
```

**Variables to rename:**
```cpp
// Internal static variables (keep descriptive names)
lastTime → last_time_us
lastCountLeft → last_count_left
lastCountRight → last_count_right
speedLeft → speed_left
speedRight → speed_right
```

**Debug calls to update:**
```cpp
debugLog() → debug_log()
debugPrintf() → debug_printf()
```

**External library objects (keep as-is):**
```cpp
encoderLeft  // ESP32Encoder library
encoderRight // ESP32Encoder library
```

---

#### 2. kinematics.cpp/h

**Functions to rename:**
```cpp
// OLD → NEW
twistToWheelSpeeds() → twist_to_wheel_speeds()
wheelSpeedsToTwist() → wheel_speeds_to_twist()
updateOdometry() → update_odometry()
```

**No internal variables to rename** (parameters are already snake_case)

---

#### 3. encoder_handler.h (needs header)

Add file header:
```cpp
/**
 * @file encoder_handler.h
 * @brief Encoder reading and speed calculation
 *
 * Interfaces with ESP32Encoder library for quadrature encoder reading.
 * Provides speed calculation in m/s based on encoder pulses.
 */
```

---

#### 4. kinematics.h (needs header)

Add file header:
```cpp
/**
 * @file kinematics.h
 * @brief Differential drive kinematics and odometry
 *
 * Implements forward and inverse kinematics for differential drive robot.
 * Provides odometry integration using encoder data.
 */
```

---

###5. main.cpp

**Function calls to update:**
```cpp
// OLD → NEW
debugSerialInit() → debug_serial_init()
debugLog() → debug_log()
debugPrintln() → debug_println()
motorInit() → motor_init()
encoderInit() → encoder_init()
pidInit() → pid_init()
resetEncoders() → reset_encoders()
twistToWheelSpeeds() → twist_to_wheel_speeds()
getWheelSpeeds() → get_wheel_speeds()
pidCompute() → pid_compute()
setMotorSpeed() → set_motor_speed()
emergencyStop() → emergency_stop()
```

**Note:** Keep ROS functions unchanged:
- `init_ros_msgs()` - ROS convention
- `ros_loop()` - ROS convention

---

### 6. ros_com.cpp

**Function calls to update:**
```cpp
// OLD → NEW (in timer_callback and other functions)
getEncoderCounts() → get_encoder_counts()
updateOdometry() → update_odometry()
getWheelSpeeds() → get_wheel_speeds()
wheelSpeedsToTwist() → wheel_speeds_to_twist()
setPIDTunings() → set_pid_tunings()
```

**Keep ROS callbacks unchanged:**
- `timer_callback()`
- `cmd_vel_callback()`
- `motor_enable_callback()`
- `pid_config_callback()`
- `create_entities()`
- `destroy_entities()`
- `init_ros_msgs()`
- `ros_loop()`

---

## Sed Commands for Batch Replacement

### encoder_handler files:
```bash
# Functions
sed -i 's/\bencoderInit\b/encoder_init/g' src/encoder_handler.cpp include/encoder_handler.h
sed -i 's/\bgetEncoderCounts\b/get_encoder_counts/g' src/encoder_handler.cpp include/encoder_handler.h
sed -i 's/\bgetWheelSpeeds\b/get_wheel_speeds/g' src/encoder_handler.cpp include/encoder_handler.h
sed -i 's/\bresetEncoders\b/reset_encoders/g' src/encoder_handler.cpp include/encoder_handler.h

# Variables (in encoder_handler.cpp only)
sed -i 's/\blastTime\b/last_time_us/g' src/encoder_handler.cpp
sed -i 's/\blastCountLeft\b/last_count_left/g' src/encoder_handler.cpp
sed -i 's/\blastCountRight\b/last_count_right/g' src/encoder_handler.cpp
sed -i 's/\bspeedLeft\b/speed_left/g' src/encoder_handler.cpp
sed -i 's/\bspeedRight\b/speed_right/g' src/encoder_handler.cpp

# Debug calls
sed -i 's/\bdebugLog\b/debug_log/g' src/encoder_handler.cpp
sed -i 's/\bdebugPrintf\b/debug_printf/g' src/encoder_handler.cpp
```

### kinematics files:
```bash
# Functions
sed -i 's/\btwistToWheelSpeeds\b/twist_to_wheel_speeds/g' src/kinematics.cpp include/kinematics.h
sed -i 's/\bwheelSpeedsToTwist\b/wheel_speeds_to_twist/g' src/kinematics.cpp include/kinematics.h
sed -i 's/\bupdateOdometry\b/update_odometry/g' src/kinematics.cpp include/kinematics.h

# Debug calls
sed -i 's/\bdebugLog\b/debug_log/g' src/kinematics.cpp
```

### main.cpp:
```bash
sed -i 's/\bdebugSerialInit\b/debug_serial_init/g' src/main.cpp
sed -i 's/\bdebugLog\b/debug_log/g' src/main.cpp
sed -i 's/\bdebugPrintln\b/debug_println/g' src/main.cpp
sed -i 's/\bmotorInit\b/motor_init/g' src/main.cpp
sed -i 's/\bencoderInit\b/encoder_init/g' src/main.cpp
sed -i 's/\bpidInit\b/pid_init/g' src/main.cpp
sed -i 's/\bresetEncoders\b/reset_encoders/g' src/main.cpp
sed -i 's/\btwistToWheelSpeeds\b/twist_to_wheel_speeds/g' src/main.cpp
sed -i 's/\bgetWheelSpeeds\b/get_wheel_speeds/g' src/main.cpp
sed -i 's/\bpidCompute\b/pid_compute/g' src/main.cpp
sed -i 's/\bsetMotorSpeed\b/set_motor_speed/g' src/main.cpp
sed -i 's/\bemergencyStop\b/emergency_stop/g' src/main.cpp
```

### ros_com.cpp:
```bash
sed -i 's/\bgetEncoderCounts\b/get_encoder_counts/g' src/ros_com.cpp
sed -i 's/\bupdateOdometry\b/update_odometry/g' src/ros_com.cpp
sed -i 's/\bgetWheelSpeeds\b/get_wheel_speeds/g' src/ros_com.cpp
sed -i 's/\bwheelSpeedsToTwist\b/wheel_speeds_to_twist/g' src/ros_com.cpp
sed -i 's/\bsetPIDTunings\b/set_pid_tunings/g' src/ros_com.cpp
```

---

## Testing Checklist

After completing refactoring:

### 1. Compilation
```bash
cd /home/max/projects/robot-drivers/waveshare-gdr-ros
pio run --target clean
pio run
```

Expected: **Zero errors, zero warnings**

### 2. Find Remaining Issues
```bash
# Find any remaining camelCase function definitions
grep -r "^void [a-z][a-zA-Z]*(" src/ include/ | grep -v "timer_callback\|cmd_vel_callback\|create_entities\|destroy_entities\|init_ros_msgs\|ros_loop\|motor_enable_callback\|pid_config_callback"

# Find camelCase variables (approximate - may have false positives)
grep -rE "^\s*(static\s+)?(float|int|double|bool|uint|int32_t)\s+[a-z][a-zA-Z]+\s*=" src/
```

### 3. Upload and Test
```bash
pio run --target upload
```

Monitor using:
```bash
# Terminal 1: micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Test commands
ros2 topic list
ros2 topic echo /ugv/odom --once
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
```

---

## Summary of Naming Conventions

| Old Style | New Style | Example |
|-----------|-----------|---------|
| `motorInit()` | `motor_init()` | Functions |
| `leftSpeed` | `left_speed` | Variables |
| `debugLog()` | `debug_log()` | Our functions |
| `pidLeft` | `pidLeft` | **External library objects - keep as-is** |
| `encoderLeft` | `encoderLeft` | **External library objects - keep as-is** |

---

## Files Summary

| File | Status | Functions | Variables | Docs |
|------|--------|-----------|-----------|------|
| docs/CODING_STYLE.md | ✅ Complete | N/A | N/A | ✅ |
| motor_control.cpp/h | ✅ Complete | 3 renamed | 0 | ✅ |
| debug_serial.cpp/h | ✅ Complete | 6 renamed | 0 | ✅ |
| pid_controller.cpp/h | ✅ Complete | 4 renamed | 6 renamed | ✅ |
| encoder_handler.cpp/h | ⏳ Pending | 4 to rename | 5 to rename | Add headers |
| kinematics.cpp/h | ⏳ Pending | 3 to rename | 0 | Add headers |
| main.cpp | ⏳ Pending | 12 calls to update | 0 | None needed |
| ros_com.cpp | ⏳ Pending | 5 calls to update | 0 | None needed |

---

## Completion Steps

1. Run sed commands above (or manually refactor)
2. Add file header documentation to encoder_handler.h and kinematics.h
3. Compile with `pio run`
4. Fix any compilation errors
5. Upload and test with micro-ROS agent
6. Verify all Phase 1 functionality works
7. Create git commit with descriptive message

---

## Git Commit Message Template

```
refactor: Apply snake_case naming convention across codebase

Apply consistent snake_case naming for all functions and variables
per new CODING_STYLE.md guidelines.

Changes:
- motor_control: motorInit() → motor_init(), setMotorSpeed() → set_motor_speed(), emergencyStop() → emergency_stop()
- debug_serial: All debug functions renamed to snake_case
- pid_controller: All PID functions and internal variables renamed
- encoder_handler: All encoder functions and variables renamed
- kinematics: All kinematic functions renamed
- main.cpp, ros_com.cpp: Updated all function calls

External library objects (pidLeft, pidRight, encoderLeft, encoderRight)
kept in camelCase to match library conventions.

ROS callback functions (timer_callback, cmd_vel_callback, etc.) kept
in snake_case per ROS2 conventions.

No functional changes - style refactoring only.

Testing: Compiled successfully, motors and encoders verified working.
```

---

**Last Updated:** 2025-10-11
**Progress:** 4/8 modules complete (50%)
