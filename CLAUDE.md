# General Driver Board - micro-ROS Implementation

## 🎯 Project Goal
Migrate Waveshare General Driver Board from JSON control to micro-ROS (ROS2 Humble).
Reference: AT_ZMOAB_ROS01 firmware (similar architecture).

## 📋 Hardware Quick Ref

### ESP32-WROOM-32 (NOT S3!)
- **USB:** Type-C with CP2102 bridge → Serial @ 115200 → /dev/ttyUSB0
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

## 📡 ROS2 Topics
```cpp
// Publishers (ESP32 → PC)
/ugv/odom          - nav_msgs/Odometry
/ugv/encoder       - std_msgs/Int32MultiArray
/ugv/imu           - sensor_msgs/Imu
/ugv/battery       - sensor_msgs/BatteryState

// Subscribers (PC → ESP32)
/cmd_vel           - geometry_msgs/Twist
/ugv/motor_enable  - std_msgs/Bool
```

## 🚀 Implementation Phases

### CURRENT PHASE: Phase 1 - Core Mobility
**Week 1-2:** Basic movement + odometry
- [ ] Motor PWM control (H-bridge, 100kHz, 8-bit)
- [ ] Encoder reading (half-quad, speed calc)
- [ ] PID control (Kp=20, Ki=2000, Kd=0)
- [ ] /cmd_vel subscriber
- [ ] /ugv/odom publisher
- [ ] Emergency stop

### Phase 2 - Sensors (Week 3)
- [ ] IMU (reuse General_Driver/QMI8658.cpp)
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
    Serial.begin(115200);  // CP2102 USB-UART
    delay(2000);
    set_microros_serial_transports(Serial);  // CRITICAL!
    
    // Init ROS
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "general_driver", "", &support);
    
    // Hardware
    motorInit();
    encoderInit();
    pidControllerInit();
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    getWheelSpeed();
    pidA.Compute();
    pidB.Compute();
    setMotorSpeed(outputA, outputB);
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

# PC: Run micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Test
ros2 topic list
ros2 topic echo /ugv/odom
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
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
- Baud MUST be 115200
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

## ✅ Phase 1 Success Criteria
- [ ] ESP32 connects to micro-ROS agent
- [ ] `ros2 node list` shows node
- [ ] Motors respond to /cmd_vel
- [ ] Encoders publish accurate data
- [ ] PID maintains speed (±10%)
- [ ] Emergency stop works

## 💡 Remember
1. **CP2102 is your friend** - Same USB port for upload & ROS
2. **Serial = micro-ROS** - Don't use for debug (use OLED or publish debug topics)
3. **ESP32 ≠ ESP32-S3** - No native USB, use Serial via CP2102
4. **Reuse tested code** - IMU from General_Driver works great
5. **Test incrementally** - Motors → Encoders → PID → ROS

---
**Start with Phase 1. Get one motor + encoder working before moving on!**