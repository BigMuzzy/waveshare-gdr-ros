# General Driver Board - micro-ROS Implementation Guide

## 🎯 Project Overview
**Goal:** Migrate Waveshare General Driver Board firmware from JSON-based control to micro-ROS (ROS2 Humble) while maintaining backward compatibility.

**Reference Implementation:** AT_ZMOAB_ROS01 board (ESP32-S3 based, similar architecture)

---

## 📋 Hardware Specifications

### General Driver Board (Waveshare)
- **MCU:** ESP32-WROOM-32 (NOT ESP32-S3)
  - 240MHz dual-core
  - 520KB SRAM
  - 4MB Flash
- **USB:** Type-C with CP2102 USB-UART bridge chip
  - Connection: USB-C → CP2102 → ESP32 GPIO1(TX)/GPIO3(RX)
  - Serial0 available at 115200 baud
  - Shows as `/dev/ttyUSB0` on Linux
- **IMU:** QMI8658C (6-axis) + AK09918 (3-axis mag) = 9-axis
- **Power Monitor:** INA219 @ I2C address 0x42
- **Display:** SSD1306 OLED 128x64 @ I2C
- **Storage:** TF card + LittleFS (flash)
- **Communication:** WiFi, Bluetooth, ESP-NOW

### Critical Differences vs AT_ZMOAB_ROS01
| Feature | General Driver (ESP32) | AT_ZMOAB_ROS01 (ESP32-S3) |
|---------|------------------------|---------------------------|
| USB | CP2102 UART bridge | Native USB OTG |
| micro-ROS | Via Serial (UART) | Via USB CDC |
| Device | /dev/ttyUSB0 | /dev/ttyACM0 |
| Max Baud | ~460800 | Much higher |
| Motor Driver | Direct H-bridge PWM | ZLAC8015D (Modbus) |

**Key Insight:** Use existing USB-C port (CP2102) for micro-ROS - same port used for firmware upload!

---

## 🔌 Pin Mapping

### Motor Control
```cpp
// Left Motor (A)
#define PWMA 25         // PWM output
#define AIN1 21         // Direction 1
#define AIN2 17         // Direction 2
#define AENCA 35        // Encoder A
#define AENCB 34        // Encoder B

// Right Motor (B)
#define PWMB 26         // PWM output
#define BIN1 22         // Direction 1
#define BIN2 23         // Direction 2
#define BENCA 27        // Encoder A
#define BENCB 16        // Encoder B

// PWM Config
int freq = 100000;      // 100kHz
int channel_A = 5;
int channel_B = 6;
const uint16_t ANALOG_WRITE_BITS = 8;  // 8-bit PWM (0-255)
```

### I2C Devices
```cpp
#define S_SDA 32
#define S_SCL 33

// I2C Addresses:
// - INA219: 0x42
// - SSD1306 OLED: 0x3C (typical)
// - QMI8658: 0x6A or 0x6B
// - AK09918: 0x0C
```

### Serial Bus Servo (ST3215)
```cpp
#define S_RXD 18        // UART2 RX
#define S_TXD 19        // UART2 TX
// Baud: 1000000 (1Mbaud)
// Can control up to 253 servos
```

### USB (micro-ROS)
```cpp
// Serial0 via CP2102
// RX: GPIO3 (internally connected)
// TX: GPIO1 (internally connected)
// Baud: 115200
```

---

## 🏗️ Project Structure

```
general_driver_ros/
├── platformio.ini
├── boards/
│   └── esp32dev.json          # Custom board definition if needed
├── include/
│   ├── config.h               # Pin definitions, constants
│   ├── ros_config.h           # ROS topic names, QoS
│   └── robot_params.h         # Wheel params, PID gains
├── src/
│   ├── main.cpp              # Main loop, ROS agent connection
│   ├── ros_com.cpp           # ROS publishers/subscribers
│   ├── ros_com.h
│   ├── motor_control.cpp     # H-bridge PWM + encoders
│   ├── motor_control.h
│   ├── pid_controller.cpp    # Closed-loop speed control
│   ├── pid_controller.h
│   ├── imu_handler.cpp       # QMI8658 + AK09918
│   ├── imu_handler.h
│   ├── battery_monitor.cpp   # INA219
│   ├── battery_monitor.h
│   ├── oled_display.cpp      # Status display
│   ├── oled_display.h
│   ├── servo_control.cpp     # ST3215 bus servos (optional)
│   ├── servo_control.h
│   └── utils.cpp             # Helper functions
├── lib/
│   ├── General_Driver/       # Reusable code from original
│   │   ├── QMI8658.cpp
│   │   ├── QMI8658.h
│   │   ├── AK09918.cpp
│   │   └── AK09918.h
│   └── SCServo/              # Servo library
└── test/
```

---

## ⚙️ platformio.ini Configuration

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

monitor_speed = 115200

build_flags = 
    -D CONFIG_IDF_TARGET_ESP32
    -D CORE_DEBUG_LEVEL=0

lib_deps =
    # micro-ROS (CRITICAL: Uses Serial via CP2102)
    https://github.com/micro-ROS/micro_ros_platformio
    
    # Motor & Sensors
    madhephaestus/ESP32Encoder @ ^0.10.1
    br3ttb/PID_v2 @ ^2.0.0
    denyssene/SimpleKalmanFilter @ ^1.0.2
    
    # Power & Display
    wollewald/INA219_WE @ ^1.3.8
    adafruit/Adafruit SSD1306 @ ^2.5.7
    adafruit/Adafruit GFX Library @ ^1.11.9
    
    # Utils
    bblanchon/ArduinoJson @ ^6.21.3
    FS
    LittleFS

# micro-ROS config - USES SERIAL (CP2102 USB-UART)
board_microros_transport = serial
board_microros_distro = humble
```

---

## 🚀 Implementation Phases

### PHASE 1: Core Mobility (Week 1-2)
**Goal:** Basic robot movement with odometry

#### 1.1 Motor Control
```cpp
// Files: motor_control.cpp, motor_control.h

- [ ] H-bridge PWM initialization (channels 5, 6)
- [ ] Direction control (AIN1/2, BIN1/2)
- [ ] Emergency stop function
- [ ] Speed limiting (0-255 range)

void motorInit();
void setMotorSpeed(int16_t left_pwm, int16_t right_pwm);
void emergencyStop();
```

#### 1.2 Encoder Reading
```cpp
// Uses ESP32Encoder library

- [ ] Half-quadrature mode setup
- [ ] Pulse counting (left/right)
- [ ] Speed calculation (pulses → m/s)
- [ ] Odometry integration

// Key parameters from AT_ZMOAB reference:
#define WHEEL_D 0.065          // 65mm diameter (adjust for your robot)
#define ONE_CIRCLE_PLUSES 180  // Pulses per revolution (adjust)
double plusesRate = 3.14159 * WHEEL_D / ONE_CIRCLE_PLUSES;

void encoderInit();
void getWheelSpeed();  // Update speed variables
```

#### 1.3 PID Control
```cpp
// Files: pid_controller.cpp
// Uses PID_v2 library

- [ ] PID initialization (Kp=20, Ki=2000, Kd=0)
- [ ] Setpoint smoothing
- [ ] Output limits (±255)
- [ ] Compute at ~100Hz

// From existing firmware:
float __kp = 20.0;
float __ki = 2000.0;
float __kd = 0;

PID_v2 pidA(__kp, __ki, __kd, PID::Direct);
PID_v2 pidB(__kp, __ki, __kd, PID::Direct);
```

#### 1.4 ROS2 Topics
```cpp
// Files: ros_com.cpp, ros_com.h

Publishers (ESP32 → PC):
  /ugv/odom          - nav_msgs/Odometry
  /ugv/encoder       - std_msgs/Int32MultiArray
  /ugv/wheel_speed   - std_msgs/Float32MultiArray

Subscribers (PC → ESP32):
  /cmd_vel           - geometry_msgs/Twist
  /ugv/motor_enable  - std_msgs/Bool
```

### PHASE 2: Sensors (Week 3)

#### 2.1 IMU (QMI8658 + AK09918)
```cpp
// Files: imu_handler.cpp
// Reuse: General_Driver/QMI8658.cpp, AK09918.cpp

- [ ] QMI8658 initialization (Acc: ±16g @ 1000Hz, Gyro: ±2048dps)
- [ ] AK09918 magnetometer initialization
- [ ] Auto-calibration (50 samples on startup)
- [ ] Sensor fusion (quaternion orientation)
- [ ] Heading calculation

// From existing code - auto-calibration:
void autoOffsets() {
    for(int i=0; i<50; i++){
        read_acc(acc);
        TempAcc.X_Off_Err += acc[0];
        // ... accumulate
    }
    TempAcc.X_Off_Err /= 50;
    TempAcc.Z_Off_Err -= 980.0;  // Gravity compensation
}

ROS Topic:
  /ugv/imu - sensor_msgs/Imu (orientation, angular_vel, linear_accel)
  /ugv/mag - sensor_msgs/MagneticField
```

#### 2.2 Battery Monitor (INA219)
```cpp
// Files: battery_monitor.cpp
// Reuse: General_Driver/battery_ctrl.h

- [ ] INA219 I2C init @ 0x42
- [ ] Shunt resistor: 0.01Ω
- [ ] Read: voltage, current, power
- [ ] Low battery threshold

void ina219_init();
void inaDataUpdate();

ROS Topic:
  /ugv/battery - sensor_msgs/BatteryState
```

#### 2.3 OLED Display
```cpp
// Files: oled_display.cpp

- [ ] SSD1306 initialization
- [ ] Display: robot name, version, battery, IP, status
- [ ] Update at 1-2 Hz

void init_oled();
void oled_update();
```

### PHASE 3: Extended Features (Week 4)

#### 3.1 Servo Control (ST3215)
```cpp
// Files: servo_control.cpp
// Uses: SCServo library

- [ ] UART2 initialization (1Mbaud)
- [ ] Position/speed/torque control
- [ ] Servo feedback (pos, load, temp, voltage)
- [ ] RoArm-M2 support (5-DOF IK)

ROS Topics:
  /ugv/joint_states - sensor_msgs/JointState
  /ugv/joint_cmd    - trajectory_msgs/JointTrajectory
```

#### 3.2 LED/Switch Control
```cpp
- [ ] 12V switch outputs (uses motor channels)
- [ ] PWM dimming for LEDs

ROS Topic:
  /ugv/led_cmd - std_msgs/UInt8MultiArray
```

#### 3.3 Backward Compatibility
```cpp
- [ ] JSON command parser (legacy)
- [ ] Dual protocol mode (ROS + JSON)
- [ ] Keep existing command structure
```

---

## 📡 micro-ROS Setup

### Main Setup (main.cpp)
```cpp
#include <micro_ros_platformio.h>

void setup() {
    // Initialize USB-C serial (CP2102)
    Serial.begin(115200);
    delay(2000);
    
    // Configure micro-ROS to use Serial (USB-UART)
    set_microros_serial_transports(Serial);
    
    // Initialize micro-ROS
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 0);
    
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_node_init_default(&node, "general_driver_robot", "", &support);
    
    // Create publishers/subscribers
    create_ros_entities();
    
    // Initialize hardware
    motorInit();
    encoderInit();
    pidControllerInit();
    imu_init();
    ina219_init();
    init_oled();
}

void loop() {
    // Spin executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    // Read sensors
    getWheelSpeed();
    updateIMUData();
    inaDataUpdate();
    
    // Compute PID
    pidA.Compute();
    pidB.Compute();
    setMotorSpeed(outputA, outputB);
    
    // Publish data
    publishOdometry();
    publishIMU();
    publishBattery();
    
    // Update display
    oled_update();
}
```

### PC Side - micro-ROS Agent
```bash
# Install agent
sudo apt install ros-humble-micro-ros-agent

# Run agent (CP2102 shows as /dev/ttyUSB0)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Test topics
ros2 topic list
ros2 topic echo /ugv/odom
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
```

---

## 🔑 Key Code Snippets to Reuse

### From General Driver (Original Firmware)
```cpp
// Motor control with PID - movtion_module.h
void getWheelSpeed() {
    unsigned long currentTime = micros();
    long encoderPulsesA = encoderA.getCount();
    speedGetA = (plusesRate * (encoderPulsesA - lastEncoderA)) / 
                ((double)(currentTime - lastTime) / 1000000);
    lastEncoderA = encoderPulsesA;
    lastTime = currentTime;
}

// IMU auto-calibration - QMI8658.cpp
void autoOffsets(void) {
    for(int i=0; i<50; i++){
        read_acc(acc);
        TempAcc.X_Off_Err += acc[0];
        TempAcc.Y_Off_Err += acc[1];
        TempAcc.Z_Off_Err += acc[2];
        delay(10);
    }
    TempAcc.X_Off_Err /= 50;
    TempAcc.Y_Off_Err /= 50;
    TempAcc.Z_Off_Err /= 50;
    TempAcc.Z_Off_Err -= 980.0;
}

// Battery monitoring - battery_ctrl.h
void inaDataUpdate() {
    shuntVoltage_mV = ina219.getShuntVoltage_mV();
    busVoltage_V = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getBusPower();
    loadVoltage_V = busVoltage_V + (shuntVoltage_mV/1000);
}
```

### From AT_ZMOAB_ROS01 (Reference)
```cpp
// ROS publisher setup - ros_com.cpp
void create_ros_entities() {
    rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/ugv/odom"
    );
    
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );
    
    rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg, 
        &cmd_vel_callback, ON_NEW_DATA
    );
}

// Odometry calculation
void publishOdometry() {
    // Calculate delta position
    double delta_left = (encoder_left - last_encoder_left) * meter_per_tick;
    double delta_right = (encoder_right - last_encoder_right) * meter_per_tick;
    
    double delta_s = (delta_left + delta_right) / 2.0;
    double delta_theta = (delta_right - delta_left) / wheel_base;
    
    // Update pose
    x += delta_s * cos(theta + delta_theta/2.0);
    y += delta_s * sin(theta + delta_theta/2.0);
    theta += delta_theta;
    
    // Publish
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    // ... quaternion from theta
    rcl_publish(&odom_pub, &odom_msg, NULL);
}
```

---

## 🛠️ Debugging & Testing

### Serial Monitor
```cpp
// Use Serial for micro-ROS, Serial for debug is not available
// Instead, use OLED or publish debug topics

// Alternative: Use Serial1 for debug if needed
Serial1.begin(115200, SERIAL_8N1, 18, 19);  // RX, TX
Serial1.println("Debug message");
```

### Common Issues
1. **Agent connection fails**
   - Check baud rate: 115200
   - Verify port: `ls /dev/ttyUSB*`
   - Try different USB cable

2. **Motors not responding**
   - Check PWM frequency (100kHz)
   - Verify H-bridge connections
   - Test with direct PWM first

3. **IMU data unstable**
   - Run auto-calibration
   - Check I2C pullups
   - Verify sensor orientation

4. **Encoder counts wrong**
   - Check encoder polarity
   - Verify half-quad mode
   - Test motor direction

---

## 📚 Resources

### Documentation
- [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_platformio)
- [General Driver Wiki](https://www.waveshare.com/wiki/General_Driver_for_Robots)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)

### Reference Repositories
- AT_ZMOAB_ROS01: `/path/to/zmoab_ros`
- General Driver Original: `/path/to/General_Driver`

### Key Libraries
- ESP32Encoder: [GitHub](https://github.com/madhephaestus/ESP32Encoder)
- PID_v2: [GitHub](https://github.com/gelraen/Arduino-PID-Library)
- INA219_WE: [GitHub](https://github.com/wollewald/INA219_WE)

---

## 🎯 Quick Start Checklist

```markdown
### Minimal Viable Robot (Phase 1)
- [ ] PlatformIO project created
- [ ] micro-ROS agent running on PC
- [ ] ESP32 connects to agent (check `ros2 node list`)
- [ ] Motors respond to `/cmd_vel`
- [ ] Encoders publish to `/ugv/encoder`
- [ ] IMU publishes to `/ugv/imu`
- [ ] Battery publishes to `/ugv/battery`
- [ ] Emergency stop works
- [ ] OLED shows status

### Test Commands
```bash
# Terminal 1: Run agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Check connection
ros2 node list
ros2 topic list

# Terminal 3: Test drive
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}' --once

# Monitor
ros2 topic echo /ugv/odom
ros2 topic echo /ugv/imu
```
```

---

## 💡 Important Notes

1. **ESP32 vs ESP32-S3**: General Driver uses original ESP32 with CP2102 UART bridge, NOT native USB
2. **Serial Port**: Use Serial (GPIO1/3) via CP2102 for micro-ROS at 115200 baud
3. **Memory**: ESP32 has less RAM than S3 - be mindful of buffer sizes
4. **Motor Driver**: Different from AT_ZMOAB (PWM vs Modbus) - can't directly copy motor code
5. **IMU**: Different sensor (QMI8658 vs BNO055) - reuse existing driver from General_Driver
6. **JSON Support**: Keep backward compatibility for existing tooling

---

## 🚦 Success Criteria

**Phase 1 Complete When:**
- Robot drives smoothly with `/cmd_vel` input
- Odometry matches actual movement (±5%)
- PID maintains target speeds (±10%)
- Emergency stop works reliably

**Phase 2 Complete When:**
- IMU data stable and accurate
- Battery monitoring accurate (±1%)
- OLED displays all status info

**Phase 3 Complete When:**
- Servos controllable via ROS
- JSON commands still work
- All sensors publishing reliably

---

**Good luck with implementation! Start with Phase 1 - basic mobility first.**