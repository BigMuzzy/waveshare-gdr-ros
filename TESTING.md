# Phase 1 Testing Guide

Firmware uploaded successfully to ESP32 at `/dev/ttyUSB0`

## Hardware Information
- **Board:** ESP32-D0WD-V3 (revision v3.1)
- **MAC:** cc:7b:5c:1f:10:f8
- **Flash:** 29.2% used (382,873 bytes)
- **RAM:** 13.9% used (45,680 bytes)

---

## Testing Procedure

### 1. Start micro-ROS Agent

In a terminal, run the micro-ROS agent to connect to the ESP32:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Expected output:**
```
[info] session established | client_key: 0x...
[info] participant created
[info] topic created (x4)
[info] publisher created (x2)
[info] subscriber created (x2)
```

---

### 2. Verify ROS Connection

Open a new terminal and check that the node and topics are available:

```bash
# Check if node is running
ros2 node list
# Expected: /general_driver

# List all topics
ros2 topic list
# Expected:
#   /cmd_vel
#   /ugv/encoder
#   /ugv/motor_enable
#   /ugv/odom
```

---

### 3. Test Basic Functionality

#### Test 1: Check Encoder Publishing
```bash
# Monitor encoder counts (should update at 20Hz)
ros2 topic echo /ugv/encoder --once

# Expected output:
# data:
# - 0      # Left encoder count
# - 0      # Right encoder count
```

#### Test 2: Check Odometry Publishing
```bash
# Monitor odometry
ros2 topic echo /ugv/odom --once

# Expected:
# header:
#   stamp: {...}
#   frame_id: odom
# child_frame_id: base_link
# pose:
#   pose:
#     position: {x: 0.0, y: 0.0, z: 0.0}
#     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
# twist:
#   twist:
#     linear: {x: 0.0, y: 0.0, z: 0.0}
#     angular: {x: 0.0, y: 0.0, z: 0.0}
```

---

### 4. Test Motor Control (CAUTION!)

⚠️ **SAFETY WARNING:**
- Ensure robot is on blocks or has wheels off the ground
- Be ready to disable motors immediately
- Start with VERY LOW speeds

#### Test 3a: Small Forward Command
```bash
# Send a very small forward velocity (0.05 m/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05}}" --once

# Monitor encoder changes
ros2 topic echo /ugv/encoder

# Expected: Encoder counts should increase
```

#### Test 3b: Stop Motors
```bash
# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}}" --once
```

#### Test 3c: Disable Motors (Emergency Stop)
```bash
# Disable motors completely
ros2 topic pub /ugv/motor_enable std_msgs/Bool "{data: false}" --once

# Re-enable motors
ros2 topic pub /ugv/motor_enable std_msgs/Bool "{data: true}" --once
```

---

### 5. Test Odometry Integration

```bash
# Terminal 1: Monitor odometry
ros2 topic echo /ugv/odom

# Terminal 2: Send velocity command for 2 seconds
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" -r 10 &
sleep 2
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}}" --once

# Check odometry position - should show ~0.2m traveled
ros2 topic echo /ugv/odom --once
```

---

### 6. Test Rotation

```bash
# Rotate in place (0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# Monitor encoder counts - should see differential movement
ros2 topic echo /ugv/encoder

# Stop rotation
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.0}}" --once
```

---

## Troubleshooting

### Motors Not Moving
1. Check `/ugv/motor_enable` - should be `true`
2. Verify PID output is non-zero (publish higher velocity)
3. Check PWM threshold (23 minimum)
4. Verify motor wiring and power supply

### Encoders Not Counting
1. Check encoder wiring (pins 35, 34, 27, 16)
2. Manually rotate wheels and check `/ugv/encoder`
3. Verify interrupt setup in `encoderInit()`

### Wrong Direction
If motors go opposite direction:
1. Edit `motor_control.cpp` - swap AIN1/AIN2 or BIN1/BIN2 logic
2. Or invert one encoder polarity in `encoder_handler.cpp`

### PID Oscillation
If motors oscillate or are unstable:
1. Reduce Kp: `pidController.h` - try Kp=10 instead of 20
2. Monitor actual vs target speed
3. Increase integral time (reduce Ki)

### Connection Issues
- Verify `/dev/ttyUSB0` exists
- Check baud rate is 115200
- Restart micro-ROS agent
- Power cycle ESP32

---

## Success Criteria

✅ **Phase 1 Complete When:**
- [ ] ESP32 connects to micro-ROS agent
- [ ] `/general_driver` node appears in `ros2 node list`
- [ ] Motors respond to `/cmd_vel` commands
- [ ] Encoders count correctly (both directions)
- [ ] Odometry updates based on encoder movement
- [ ] PID maintains target speed (±10%)
- [ ] Emergency stop works (`/ugv/motor_enable false`)
- [ ] Turning works (rotation updates theta)

---

## PID Tuning (If Needed)

Current gains (in `pid_controller.h`):
- Kp = 20.0
- Ki = 2000.0
- Kd = 0.0

If performance is poor:
1. **Too slow to reach target:** Increase Kp or Ki
2. **Oscillates/overshoots:** Decrease Kp
3. **Steady-state error:** Increase Ki
4. **Noisy/jittery:** Add small Kd (0.1-1.0)

Edit `include/pid_controller.h` and rebuild:
```bash
pio run --target upload
```

---

## Next Steps

After successful Phase 1 testing:
- **Phase 2:** Add IMU (sensor_msgs/Imu) and Battery (sensor_msgs/BatteryState)
- **Phase 3:** Tune odometry parameters (wheel diameter, wheelbase)
- **Navigation:** Integrate with ROS2 navigation stack
- **Testing:** Drive precise distances to validate odometry accuracy
