# Phase 1 Implementation - Motor Control & Odometry

## 🎯 Objective
Complete Phase 1 of the General Driver micro-ROS implementation by replacing all mocked/stubbed code with real hardware drivers for motor control, encoders, and odometry.

## 📊 Current Status Analysis

### ✅ Already Implemented
- **ROS Infrastructure** (`src/ros_com.cpp`, `src/ros_com.h`)
  - Connection state machine (WAITING_AGENT, AGENT_CONNECTED, etc.)
  - Publishers: `/ugv/odom`, `/ugv/encoder`
  - Subscribers: `/cmd_vel`, `/ugv/motor_enable`
  - Timer callback (50ms period)
  - Message initialization

- **Main Loop** (`src/main.cpp`)
  - micro-ROS serial transport setup
  - Basic structure with TODOs

### ❌ Currently Mocked (NEED TO IMPLEMENT)

#### 1. Motor Control (CRITICAL)
**Location:** `src/ros_com.cpp` lines ~100-115 (timer_callback)
**Current:** Mock integration of cmd_vel
```cpp
// MOCKED - Just integrates cmd_vel
mock_odom_x += cmd_linear_x * cos(mock_odom_theta) * dt;
mock_encoder_left += (int32_t)(cmd_linear_x * dt * pulses_per_meter);
```

**Need:** Real H-bridge PWM motor driver

#### 2. Encoder Reading (CRITICAL)
**Current:** Fake encoder increments based on cmd_vel
**Need:** ESP32Encoder library reading real pulses

#### 3. PID Control (CRITICAL)
**Current:** None - direct velocity commands
**Need:** Closed-loop speed control with PID

#### 4. Differential Drive Kinematics (CRITICAL)
**Current:** Simplified mock odometry
**Need:** Proper differential drive with wheel base

---

## 🔧 Implementation Tasks

### Task 1: Create Motor Control Module
**Files to Create:**
- `include/motor_control.h`
- `src/motor_control.cpp`

**Requirements:**
```cpp
// motor_control.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Pin definitions (from reference/general_driver/ugv_config.h)
#define PWMA 25
#define AIN1 21
#define AIN2 17
#define PWMB 26
#define BIN1 22
#define BIN2 23

// PWM configuration
#define PWM_FREQ 100000      // 100kHz
#define PWM_CHANNEL_A 5
#define PWM_CHANNEL_B 6
#define PWM_RESOLUTION 8     // 8-bit (0-255)

// Functions to implement
void motorInit();
void setMotorSpeed(int16_t left_pwm, int16_t right_pwm);
void emergencyStop();

#endif
```

**Implementation Guide:**
1. Initialize PWM channels with `ledcSetup()` and `ledcAttachPin()`
2. Implement direction control using AIN1/AIN2 and BIN1/BIN2
3. Handle positive/negative PWM values (direction + magnitude)
4. Add safety limits (max ±255)

**Reference Code:** `reference/general_driver/movtion_module.h`
- Look for `movtionPinInit()` - PWM setup pattern
- Look for `leftCtrl()`, `rightCtrl()` - direction control logic

---

### Task 2: Create Encoder Module
**Files to Create:**
- `include/encoder_handler.h`
- `src/encoder_handler.cpp`

**Requirements:**
```cpp
// encoder_handler.h
#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <ESP32Encoder.h>

// Pin definitions
#define AENCA 35
#define AENCB 34
#define BENCA 27
#define BENCB 16

// Robot parameters (ADJUST FOR YOUR ROBOT!)
#define WHEEL_DIAMETER 0.065      // meters (65mm)
#define PULSES_PER_REV 180        // encoder pulses per wheel revolution
#define WHEEL_BASE 0.200          // meters (distance between wheels)

// Functions to implement
void encoderInit();
void getEncoderCounts(int32_t &left, int32_t &right);
void getWheelSpeeds(float &left_speed, float &right_speed);
void resetEncoders();

// Global encoder objects (extern in .h, define in .cpp)
extern ESP32Encoder encoderLeft;
extern ESP32Encoder encoderRight;

#endif
```

**Implementation Guide:**
1. Use `attachHalfQuad()` for encoder setup
2. Track previous counts and timestamps
3. Calculate speed: `speed = (delta_pulses / delta_time) * (π * diameter / pulses_per_rev)`
4. Handle direction (left encoder may be inverted)

**Reference Code:** `reference/general_driver/movtion_module.h`
- Look for `initEncoders()` - setup pattern
- Look for `getWheelSpeed()` - speed calculation formula
- Note: `plusesRate = 3.14159 * WHEEL_D / ONE_CIRCLE_PLUSES`

---

### Task 3: Create PID Controller Module
**Files to Create:**
- `include/pid_controller.h`
- `src/pid_controller.cpp`

**Requirements:**
```cpp
// pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <PID_v2.h>

// PID gains (from reference/general_driver/ugv_config.h)
#define PID_KP 20.0
#define PID_KI 2000.0
#define PID_KD 0.0
#define PID_OUTPUT_MIN -255
#define PID_OUTPUT_MAX 255

// Functions to implement
void pidInit();
void pidCompute(float left_setpoint, float right_setpoint, 
                float left_speed, float right_speed,
                int16_t &left_output, int16_t &right_output);
void pidReset();

// Global PID objects (extern in .h, define in .cpp)
extern PID_v2 pidLeft;
extern PID_v2 pidRight;

#endif
```

**Implementation Guide:**
1. Create two PID controllers (left/right wheels)
2. Initialize with `Start()` method
3. Set output limits to ±255 (PWM range)
4. Call `Compute()` in control loop (~100Hz)
5. Input = actual speed (m/s), Output = PWM value

**Reference Code:** `reference/general_driver/movtion_module.h`
- Look for `pidControllerInit()` - PID setup
- Look for `LeftPidControllerCompute()` - compute pattern

---

### Task 4: Implement Differential Drive Kinematics
**Files to Create:**
- `include/kinematics.h`
- `src/kinematics.cpp`

**Requirements:**
```cpp
// kinematics.h
#ifndef KINEMATICS_H
#define KINEMATICS_H

// Convert twist (linear_x, angular_z) to wheel speeds (m/s)
void twistToWheelSpeeds(float linear_x, float angular_z, 
                        float &left_speed, float &right_speed);

// Convert wheel speeds to twist
void wheelSpeedsToTwist(float left_speed, float right_speed,
                        float &linear_x, float &angular_z);

// Update odometry from wheel encoder increments
void updateOdometry(int32_t left_delta, int32_t right_delta,
                   float &x, float &y, float &theta);

#endif
```

**Implementation Guide:**
1. **Twist to Wheel Speeds:**
   ```cpp
   left_speed = linear_x - (angular_z * WHEEL_BASE / 2.0);
   right_speed = linear_x + (angular_z * WHEEL_BASE / 2.0);
   ```

2. **Wheel Speeds to Twist:**
   ```cpp
   linear_x = (left_speed + right_speed) / 2.0;
   angular_z = (right_speed - left_speed) / WHEEL_BASE;
   ```

3. **Odometry Update:**
   ```cpp
   // Convert pulses to meters
   float left_meters = left_delta * (M_PI * WHEEL_DIAMETER / PULSES_PER_REV);
   float right_meters = right_delta * (M_PI * WHEEL_DIAMETER / PULSES_PER_REV);
   
   // Calculate robot displacement
   float delta_s = (left_meters + right_meters) / 2.0;
   float delta_theta = (right_meters - left_meters) / WHEEL_BASE;
   
   // Update pose
   x += delta_s * cos(theta + delta_theta / 2.0);
   y += delta_s * sin(theta + delta_theta / 2.0);
   theta += delta_theta;
   ```

**Reference Code:** `reference/zmoab_ros01/src/motor_control.cpp`
- Look for odometry calculation pattern
- Note: Watch for encoder direction (may need to invert one side)

---

### Task 5: Update Main Loop
**File to Modify:** `src/main.cpp`

**Replace TODOs with:**

```cpp
#include "motor_control.h"
#include "encoder_handler.h"
#include "pid_controller.h"
#include "kinematics.h"

// Global variables
float target_left_speed = 0.0;
float target_right_speed = 0.0;
int32_t last_encoder_left = 0;
int32_t last_encoder_right = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);
    set_microros_serial_transports(Serial);
    delay(2000);

    init_ros_msgs();

    // Initialize hardware
    motorInit();
    encoderInit();
    pidInit();
    
    // Zero encoders
    resetEncoders();
}

void loop() {
    ros_loop();  // Handle ROS communication
    
    // Control loop (runs when agent connected)
    if (state_connected) {
        // 1. Convert cmd_vel to wheel speeds
        twistToWheelSpeeds(cmd_linear_x, cmd_angular_z, 
                          target_left_speed, target_right_speed);
        
        // 2. Read actual wheel speeds
        float actual_left_speed, actual_right_speed;
        getWheelSpeeds(actual_left_speed, actual_right_speed);
        
        // 3. PID control
        int16_t left_pwm, right_pwm;
        pidCompute(target_left_speed, target_right_speed,
                   actual_left_speed, actual_right_speed,
                   left_pwm, right_pwm);
        
        // 4. Apply motor commands (if enabled)
        if (motor_enabled) {
            setMotorSpeed(left_pwm, right_pwm);
        } else {
            emergencyStop();
        }
    }
}
```

---

### Task 6: Update ROS Publishing
**File to Modify:** `src/ros_com.cpp`

**In `timer_callback()` - Replace mock data:**

```cpp
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    if (timer != NULL) {
        rcl_ret_t ret = RCL_RET_OK;

        // Get real encoder counts
        int32_t current_left, current_right;
        getEncoderCounts(current_left, current_right);
        
        // Calculate deltas
        int32_t delta_left = current_left - last_encoder_left;
        int32_t delta_right = current_right - last_encoder_right;
        last_encoder_left = current_left;
        last_encoder_right = current_right;
        
        // Update odometry (defined as global in kinematics)
        static float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;
        updateOdometry(delta_left, delta_right, odom_x, odom_y, odom_theta);
        
        // Publish Odometry
        odom_msg.header.stamp.sec = (int32_t)(millis() / 1000);
        odom_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
        
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0);
        odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0);
        
        // Get current wheel speeds for twist
        float left_speed, right_speed;
        getWheelSpeeds(left_speed, right_speed);
        float linear_x, angular_z;
        wheelSpeedsToTwist(left_speed, right_speed, linear_x, angular_z);
        
        odom_msg.twist.twist.linear.x = linear_x;
        odom_msg.twist.twist.angular.z = angular_z;
        
        ret = rcl_publish(&odom_pub, &odom_msg, NULL);
        
        // Publish Encoder counts
        encoder_msg.data.data[0] = current_left;
        encoder_msg.data.data[1] = current_right;
        ret = rcl_publish(&encoder_pub, &encoder_msg, NULL);
    }
}
```

**Add global variables at top of file:**
```cpp
static int32_t last_encoder_left = 0;
static int32_t last_encoder_right = 0;
```

---

## 📝 Implementation Checklist

### Step-by-Step Order:

1. **[ ] Create `include/motor_control.h`**
   - Define pins and constants
   - Declare functions

2. **[ ] Create `src/motor_control.cpp`**
   - Implement `motorInit()` with PWM setup
   - Implement `setMotorSpeed()` with direction logic
   - Implement `emergencyStop()`
   - Reference: `reference/general_driver/movtion_module.h`

3. **[ ] Create `include/encoder_handler.h`**
   - Define pins and robot parameters
   - Declare functions

4. **[ ] Create `src/encoder_handler.cpp`**
   - Implement `encoderInit()` with half-quad setup
   - Implement `getEncoderCounts()`
   - Implement `getWheelSpeeds()` with speed calculation
   - Reference: `reference/general_driver/movtion_module.h`

5. **[ ] Create `include/pid_controller.h`**
   - Define PID gains
   - Declare PID objects and functions

6. **[ ] Create `src/pid_controller.cpp`**
   - Implement `pidInit()` with PID_v2 setup
   - Implement `pidCompute()` for both wheels
   - Reference: `reference/general_driver/movtion_module.h`

7. **[ ] Create `include/kinematics.h`**
   - Declare conversion functions

8. **[ ] Create `src/kinematics.cpp`**
   - Implement `twistToWheelSpeeds()`
   - Implement `wheelSpeedsToTwist()`
   - Implement `updateOdometry()`
   - Reference: `reference/zmoab_ros01/src/motor_control.cpp`

9. **[ ] Update `src/main.cpp`**
   - Add includes for new modules
   - Initialize hardware in setup()
   - Implement control loop in loop()

10. **[ ] Update `src/ros_com.cpp`**
    - Replace mock data in `timer_callback()`
    - Use real encoder counts
    - Calculate real odometry

11. **[ ] Test incrementally:**
    - Motors: Test PWM output with simple commands
    - Encoders: Verify counts change when wheels turn
    - PID: Check setpoint tracking
    - Odometry: Verify position updates correctly

---

## 🚨 Critical Points

### Motor Direction
- **Left motor may be reversed!** If robot goes backward when it should go forward, swap AIN1/AIN2 logic or negate PWM
- **Test each motor individually first** before combining

### Encoder Polarity
- **One encoder may be inverted** - check sign of counts
- If robot turns wrong direction, invert one encoder reading

### PID Tuning
- **Start with Kp=20, Ki=2000, Kd=0** (from original firmware)
- If oscillating: reduce Kp
- If slow response: increase Kp or Ki
- Monitor with: `ros2 topic echo /ugv/odom`

### Wheel Parameters
- **CRITICAL: Measure your actual robot!**
  - Wheel diameter (WHEEL_DIAMETER)
  - Pulses per revolution (PULSES_PER_REV)
  - Wheel base / track width (WHEEL_BASE)
- **Test odometry:** Drive 1 meter, check if odom shows ~1 meter

### Safety
- **Always implement `emergencyStop()`** - set all pins LOW
- **Check `motor_enabled` flag** before applying PWM
- **Limit PWM output** to safe range (maybe ±200 for initial testing)

---

## 🧪 Testing Procedure

### Test 1: Motor PWM (No ROS)
```cpp
void setup() {
    motorInit();
    setMotorSpeed(100, 100);  // Both forward at 100 PWM
    delay(2000);
    setMotorSpeed(-100, -100); // Both reverse
    delay(2000);
    emergencyStop();
}
```

### Test 2: Encoders (Serial Monitor)
```cpp
void loop() {
    int32_t left, right;
    getEncoderCounts(left, right);
    Serial.printf("Left: %d, Right: %d\n", left, right);
    delay(100);
}
```

### Test 3: Full System with ROS
```bash
# Terminal 1: Agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Test
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once

# Terminal 3: Monitor
ros2 topic echo /ugv/odom
ros2 topic echo /ugv/encoder
```

---

## 📚 Key Reference Files

### From General Driver (Hardware)
- `reference/general_driver/ugv_config.h` - Pin definitions
- `reference/general_driver/movtion_module.h` - Motor & encoder code
- `reference/general_driver/movtion_module.h` - PID setup

### From AT_ZMOAB_ROS01 (ROS Structure)
- `reference/zmoab_ros01/src/motor_control.cpp` - Odometry calculation
- `reference/zmoab_ros01/src/ros_com.cpp` - Publishing patterns

### Project Files
- `CLAUDE.md` - Quick reference
- `adr/general driver board - micro-rOS implementation guide.md` - Full guide

---

## ✅ Success Criteria

**Phase 1 Complete When:**
- [ ] Motors respond to `/cmd_vel` commands
- [ ] Encoders count correctly (both directions)
- [ ] PID maintains target speed (±10%)
- [ ] Odometry accumulates correctly (test: drive 1m straight, check odom)
- [ ] Turning works (test: rotate 90°, check odom theta)
- [ ] Emergency stop works via `/ugv/motor_enable false`
- [ ] No oscillation or instability

**Verification Commands:**
```bash
# Drive forward 0.1 m/s for 5 seconds
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" -r 10 &
sleep 5
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}}" --once

# Check odometry
ros2 topic echo /ugv/odom --once
# Should show ~0.5m traveled

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once
```

---

**Start with Task 1 (Motor Control). Test each module before moving to next!**