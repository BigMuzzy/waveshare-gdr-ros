# General Driver Board - micro-ROS Implementation

## 🎯 Project Goal
Migrate Waveshare General Driver Board from JSON control to micro-ROS (ROS2 Humble).
Reference: AT_ZMOAB_ROS01 firmware (similar architecture).

## ✅ Current Status
**Phase 1 & 2 COMPLETE** - Fully functional differential drive robot with:
- ✅ Motor control with PID speed regulation (Kp=20, Ki=2000)
- ✅ Wheel odometry publishing @ 50 Hz
- ✅ Encoder counts publishing @ 50 Hz
- ✅ IMU (QMI8658C) publishing @ 50 Hz
- ✅ Safety watchdog (500ms cmd_vel timeout)
- ✅ Dynamic PID tuning via ROS topic
- ✅ Optimized serial @ 2 Mbaud for reliable 50 Hz operation

## 📋 Hardware Quick Ref

### ESP32-WROOM-32 (NOT S3!)
- **USB:** Type-C with CP2102 bridge → Serial @ 2Mbaud → /dev/ttyUSB0
- **micro-ROS:** Uses Serial (UART), NOT native USB
- **I2C:** SDA(32), SCL(33)
- **IMU:** QMI8658C + AK09918 (9-axis)
- **Power:** INA219 @ 0x42

### Critical Pin Map
```cpp
// Motors
#define PWMA 25, AIN1 21, AIN2 17     // Left motor
#define PWMB 26, BIN1 22, BIN2 23     // Right motor
#define AENCA 35, AENCB 34             // Left encoder
#define BENCA 27, BENCB 16             // Right encoder

// I2C: SDA 32, SCL 33
// Servo UART: RXD 18, TXD 19 @ 1Mbaud
```

## 🏗️ Project Structure
```
src/
├── main.cpp              # ROS agent connection, main loop
├── ros_com.cpp/h         # Publishers/subscribers
├── motor_control.cpp/h   # H-bridge PWM + encoders
├── pid_controller.cpp/h  # Speed control
├── imu_handler.cpp/h     # QMI8658 + AK09918
├── battery_monitor.cpp/h # INA219
└── oled_display.cpp/h    # Status display

reference/general_driver/       # Reuse: QMI8658, AK09918, battery_ctrl
```

## ⚙️ platformio.ini Key Config
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200

lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
    madhephaestus/ESP32Encoder
    br3ttb/PID_v2
    wollewald/INA219_WE
    adafruit/Adafruit SSD1306
    bblanchon/ArduinoJson

board_microros_transport = serial  # Uses CP2102!
board_microros_distro = humble
```

## 📡 ROS2 Topics (50 Hz)
```cpp
// Publishers (ESP32 → PC) @ 50 Hz
/ugv/odom          - nav_msgs/Odometry           (50 Hz)
/ugv/encoder       - std_msgs/Int32MultiArray    (50 Hz)
/ugv/imu           - sensor_msgs/Imu             (50 Hz)
/ugv/battery       - sensor_msgs/BatteryState    (Future)

// Subscribers (PC → ESP32)
/cmd_vel           - geometry_msgs/Twist
/ugv/motor_enable  - std_msgs/Bool
/ugv/pid_config    - std_msgs/Float32MultiArray  (Kp, Ki, Kd)
```

## 🚀 Implementation Phases

### Phase 1 - Core Mobility ✅ COMPLETE
**Week 1-2:** Basic movement + odometry
- [x] Motor PWM control (H-bridge, 100kHz, 8-bit)
- [x] Encoder reading (half-quad, speed calc)
- [x] PID control (Kp=20, Ki=2000, Kd=0) with dynamic tuning
- [x] /cmd_vel subscriber with 500ms timeout watchdog
- [x] /ugv/odom publisher @ 50 Hz
- [x] /ugv/encoder publisher @ 50 Hz
- [x] Emergency stop
- [x] **Safety: cmd_vel timeout (500ms) - motors auto-stop**

### CURRENT PHASE: Phase 2 - Sensors ✅ COMPLETE
**Week 3:** IMU integration
- [x] IMU (QMI8658C) with auto-calibration
- [x] /ugv/imu publisher @ 50 Hz
- [x] Serial optimized @ 2 Mbaud for 50 Hz publishing
- [ ] Battery monitor (INA219)
- [ ] OLED display

### Phase 3 - Extended (Week 4)
- [ ] Servo control (ST3215)
- [ ] LED/switches
- [ ] JSON backward compat

## 🔑 Critical Code Patterns

### micro-ROS Setup (main.cpp)
```cpp
void setup() {
    Serial.begin(2000000);  // CP2102 USB-UART @ 2 Mbaud
    delay(2000);
    set_microros_serial_transports(Serial);  // CRITICAL!

    // Init ROS
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "general_driver", "", &support);

    // Hardware
    motor_init();
    encoder_init();
    pid_init();
    imu_init();
    imu_calibrate();  // Auto-calibration (robot must be stationary!)
}

void loop() {
    ros_loop();  // State machine + executor (5ms timeout, 50 Hz timer)

    // Control loop runs at ~1000 Hz
    if (state_connected) {
        if (is_cmd_vel_timeout()) {
            emergency_stop();
            return;
        }
        twist_to_wheel_speeds(cmd_linear_x, cmd_angular_z, target_left, target_right);
        get_wheel_speeds(actual_left, actual_right);
        pid_compute(target_left, target_right, actual_left, actual_right, pwm_left, pwm_right);
        if (motor_enabled) set_motor_speed(pwm_left, pwm_right);
    }
}
```

### Motor Control (from General_Driver)
```cpp
// Reuse pattern from movtion_module.h
void getWheelSpeed() {
    unsigned long t = micros();
    long pulseA = encoderA.getCount();
    speedA = (plusesRate * (pulseA - lastPulseA)) / ((t - lastTime) / 1e6);
    lastPulseA = pulseA;
    lastTime = t;
}

// PID params
float __kp = 20.0, __ki = 2000.0, __kd = 0;
#define WHEEL_D 0.065
#define ONE_CIRCLE_PULSES 180
double plusesRate = 3.14159 * WHEEL_D / ONE_CIRCLE_PULSES;
```

### IMU (reuse from lib/General_Driver/)
```cpp
// QMI8658.cpp - auto-calibration on startup
void autoOffsets() {
    for(int i=0; i<50; i++) {
        read_acc(acc);
        TempAcc.X_Off_Err += acc[0];
        // ... average 50 samples
    }
    TempAcc.Z_Off_Err -= 980.0;  // Gravity comp
}
```

## 🛠️ Common Commands
```bash
# Build & Upload
pio run --target upload

# Monitor (note: Serial used by micro-ROS!)
pio device monitor

# PC: Run micro-ROS agent @ 2 Mbaud
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 2000000

# Test topics
ros2 topic list
ros2 topic hz /ugv/odom      # Should show ~50 Hz
ros2 topic hz /ugv/encoder   # Should show ~50 Hz
ros2 topic hz /ugv/imu       # Should show ~50 Hz
ros2 topic echo /ugv/odom

# Control robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"

# Tune PID on-the-fly (Kp, Ki, Kd)
ros2 topic pub /ugv/pid_config std_msgs/Float32MultiArray "{data: [20.0, 2000.0, 0.0]}"
```

## ⚠️ Critical Differences from AT_ZMOAB_ROS01

| Feature | General Driver | AT_ZMOAB_ROS01 |
|---------|---------------|----------------|
| USB | CP2102 bridge | Native USB OTG |
| Serial | /dev/ttyUSB0 | /dev/ttyACM0 |
| micro-ROS | Serial UART | USB CDC |
| Motor | H-bridge PWM | ZLAC8015D Modbus |
| IMU | QMI8658 | BNO055 |

**⚠️ NEVER copy motor driver code from AT_ZMOAB - completely different!**
**✅ DO reuse ROS structure, odometry logic, and general patterns**

## 🐛 Debugging Tips

### Connection Issues
- Check: `ls /dev/ttyUSB*` (should see ttyUSB0)
- Baud MUST be 2000000 (2 Mbaud) for 50 Hz operation
- Use `set_microros_serial_transports(Serial)` not USB!

### Motor Issues
- Verify PWM freq: 100kHz (100000)
- Test with simple PWM before PID
- Check H-bridge wiring

### IMU Unstable
- Run auto-calibration (50 samples)
- Check I2C: `i2cdetect -y 1`
- Verify QMI8658 @ 0x6A or 0x6B

## 📚 Key Files to Reference
- **Full guide:** `docs/IMPLEMENTATION.md` (if created)
- **Original firmware:** `lib/General_Driver/` (reuse IMU, battery)
- **ROS reference:** AT_ZMOAB_ROS01 (structure only!)

## Reference Code
- `reference/zmoab_ros01/` - AT_ZMOAB_ROS01 for ROS patterns
- `reference/general_driver/` - Original firmware for hardware drivers

## ⚙️ Performance Tuning & Frequency Optimization

### Achieved Frequencies
| Component | Frequency | Notes |
|-----------|-----------|-------|
| **ROS Publishers** | 50 Hz | All topics (odom, encoder, IMU) |
| **Main control loop** | ~1000 Hz | Continuous execution |
| **PID computation** | ~1000 Hz | Every control loop |
| **Motor PWM** | 100 kHz | Hardware LEDC peripheral |
| **Serial baud** | 2 Mbaud | Optimized for bandwidth |

### Frequency Tuning Lessons
**Why 50 Hz (not 100 Hz)?**
- CP2102 serial bandwidth limit with 3 simultaneous topics
- I2C IMU reads (~12 bytes) add ~10ms overhead per callback
- micro-ROS executor overhead on ESP32 @ 240 MHz
- 50 Hz is excellent for mobile robotics (standard in industry)

**Critical timing parameters** (`config.h`):
```cpp
ROS::TIMER_PERIOD_MS = 20;           // 50 Hz publish rate
ROS::EXECUTOR_SPIN_TIMEOUT_MS = 5;   // Must be << TIMER_PERIOD_MS
UART::BAUD_RATE = 2000000;           // 2 Mbaud for sufficient bandwidth
```

**Bottleneck analysis:**
- 10ms timer + 100ms executor timeout = 44 Hz ❌
- 10ms timer + 1ms executor timeout (no IMU) = 66 Hz
- 10ms timer + 1ms executor timeout (with IMU) = 77 Hz
- 20ms timer + 5ms executor timeout (with IMU) = **50 Hz** ✅

## ✅ Success Criteria
- [x] ESP32 connects to micro-ROS agent
- [x] `ros2 node list` shows node
- [x] Motors respond to /cmd_vel
- [x] Encoders publish accurate data
- [x] PID maintains speed (±10%)
- [x] Emergency stop works
- [x] **cmd_vel timeout safety (500ms)**
- [x] **All topics publish at stable 50 Hz**

## 💡 Remember
1. **CP2102 is your friend** - Same USB port for upload & ROS
2. **Serial = micro-ROS** - Don't use for debug (use OLED or publish debug topics)
3. **ESP32 ≠ ESP32-S3** - No native USB, use Serial via CP2102
4. **Reuse tested code** - IMU from General_Driver works great
5. **Test incrementally** - Motors → Encoders → PID → ROS

---
**Start with Phase 1. Get one motor + encoder working before moving on!**