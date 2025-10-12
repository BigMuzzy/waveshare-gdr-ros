#include "ros_com.h"
#include "encoder_handler.h"
#include "kinematics.h"
#include "debug_serial.h"
#include "config.h"
#include "pid_controller.h"
#include "imu_handler.h"

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state, prev_state;

bool state_connected = false;

// Command data from subscribers
float cmd_linear_x = 0.0;
float cmd_angular_z = 0.0;
bool motor_enabled = true;

// Command velocity timeout tracking (safety feature)
static unsigned long last_cmd_vel_time_ms = 0;  // Timestamp of last received cmd_vel

// Odometry tracking variables (real hardware data)
static float odom_x = 0.0;          // Global X position (m)
static float odom_y = 0.0;          // Global Y position (m)
static float odom_theta = 0.0;      // Global heading (rad)
static int32_t last_encoder_left = 0;   // Previous left encoder count
static int32_t last_encoder_right = 0;  // Previous right encoder count

/* --------------------------- */
/* --- RCL objects Declare --- */
/* --------------------------- */
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/* -------------------------- */
/* --- ROS Topics Declare --- */
/* -------------------------- */
// Publishers
rcl_publisher_t odom_pub;
rcl_publisher_t encoder_pub;
rcl_publisher_t imu_pub;

// Subscribers
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t motor_enable_sub;
rcl_subscription_t pid_config_sub;

/* ------------------------ */
/* --- ROS Msg Declare --- */
/* ----------------------- */
// Publisher messages
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32MultiArray encoder_msg;
sensor_msgs__msg__Imu imu_msg;

// Subscriber messages
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Bool motor_enable_msg;
std_msgs__msg__Float32MultiArray pid_config_msg;

// PID config storage
static float pid_gains[3];

/* -------------------- */
/* --- ROS Msg init --- */
/* -------------------- */
void init_ros_msgs() {

    // Initialize odometry message (frame IDs from config.h)
    odom_msg.header.frame_id.data = const_cast<char*>(ROS::FRAME_ID_ODOM);
    odom_msg.child_frame_id.data = const_cast<char*>(ROS::FRAME_ID_BASE_LINK);

    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Initialize covariance arrays (size from config.h)
    for (int i = 0; i < ROS::COVARIANCE_ARRAY_SIZE; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }

    // Initialize encoder message (size from config.h)
    static int32_t encoder_data[ROS::ENCODER_ARRAY_SIZE] = {0, 0};
    encoder_msg.data.capacity = ROS::ENCODER_ARRAY_SIZE;
    encoder_msg.data.size = ROS::ENCODER_ARRAY_SIZE;
    encoder_msg.data.data = encoder_data;

    // Initialize subscriber messages
    motor_enable_msg.data = true;

    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    // Initialize PID config message
    pid_config_msg.data.capacity = 3;
    pid_config_msg.data.size = 3;
    pid_config_msg.data.data = pid_gains;

    // Initialize IMU message
    imu_msg.header.frame_id.data = const_cast<char*>(ROS::FRAME_ID_BASE_LINK);

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;

    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    // Initialize covariance matrices (9 elements each)
    // Set -1 for unknown orientation covariance (no magnetometer/fusion)
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = -1.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
}

/* ------------------------ */
/* --- ROS Sub Callback --- */
/* ------------------------ */
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    cmd_linear_x = msg->linear.x;
    cmd_angular_z = msg->angular.z;

    // Update timestamp for timeout watchdog
    last_cmd_vel_time_ms = millis();
}

void motor_enable_callback(const void *msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    motor_enabled = msg->data;

    // Stop motors if disabled
    if (!motor_enabled) {
        cmd_linear_x = 0.0;
        cmd_angular_z = 0.0;
    }
}

void pid_config_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

    float kp = msg->data.data[0];
    float ki = msg->data.data[1];
    float kd = msg->data.data[2];

    set_pid_tunings(kp, ki, kd);
}

/* -------------------------- */
/* --- IMU Data Publisher --- */
/* -------------------------- */
void publish_imu_data() {
    // Check if IMU is ready before attempting to read
    if (!is_imu_ready()) {
        return;
    }

    // Read IMU sensor data
    float accel[3], gyro[3];
    if (!imu_read(accel, gyro)) {
        // Read failed, skip this update
        return;
    }

    // Update timestamp
    int64_t time_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

    // Update linear acceleration (m/s²)
    imu_msg.linear_acceleration.x = accel[0];
    imu_msg.linear_acceleration.y = accel[1];
    imu_msg.linear_acceleration.z = accel[2];

    // Update angular velocity (rad/s)
    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];

    // Orientation is not computed (no magnetometer/fusion)
    // Quaternion remains identity: (0, 0, 0, 1)
    // Covariance is -1 to indicate "unknown" per ROS convention

    // Publish IMU message
    rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
    (void)ret; // Suppress unused variable warning
}

/* -------------------------- */
/* --- ROS Timer Callback --- */
/* -------------------------- */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

    (void) last_call_time;
    if (timer != NULL) {

        rcl_ret_t ret = RCL_RET_OK;

        // Get real encoder counts from hardware
        int32_t current_left, current_right;
        get_encoder_counts(current_left, current_right);

        // Calculate encoder deltas since last update
        int32_t delta_left = current_left - last_encoder_left;
        int32_t delta_right = current_right - last_encoder_right;

        // Update odometry using real encoder data and kinematics
        update_odometry(delta_left, delta_right, odom_x, odom_y, odom_theta);

        // Save current encoder counts for next iteration
        last_encoder_left = current_left;
        last_encoder_right = current_right;

        // Get current wheel speeds for twist velocity
        float left_speed, right_speed;
        get_wheel_speeds(left_speed, right_speed);

        // Convert wheel speeds to twist (linear_x, angular_z)
        float linear_x, angular_z;
        wheel_speeds_to_twist(left_speed, right_speed, linear_x, angular_z);

        // Publish Odometry with real data
        int64_t time_ns = rmw_uros_epoch_nanos();
        odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
        odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;

        // Convert theta to quaternion
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0);
        odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0);

        // Publish actual velocity from encoders (not commanded velocity)
        odom_msg.twist.twist.linear.x = linear_x;
        odom_msg.twist.twist.angular.z = angular_z;

        ret = rcl_publish(&odom_pub, &odom_msg, NULL);

        // Publish real encoder counts
        encoder_msg.data.data[0] = current_left;
        encoder_msg.data.data[1] = current_right;
        ret = rcl_publish(&encoder_pub, &encoder_msg, NULL);

        // Publish IMU data
        publish_imu_data();
    }
}

////////////////////////////////////
/// ROS Init & Destroy Functions ///
////////////////////////////////////
bool create_entities() {

    // Sync session timeout (from config.h)
    rmw_uros_sync_session(ROS::SESSION_TIMEOUT_MS);

    // Node configuration (from config.h)
    const char * node_name = ROS::NODE_NAME;
    const char * ns = ROS::NAMESPACE;
    const int domain_id = ROS::DOMAIN_ID;

    // Initialize node
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = RCL_RET_OK;
    ret = rcl_init_options_init(&init_options, allocator);
    ret = rcl_init_options_set_domain_id(&init_options, domain_id);
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_node_init_default(&node, node_name, ns, &support);

    /* ----------------- */
    /* --- Publisher --- */
    /* ----------------- */

    ret = rclc_publisher_init(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        ROS::TOPIC_ODOM,
        &rmw_qos_profile_default);

    rclc_publisher_init(
        &encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        ROS::TOPIC_ENCODER,
        &rmw_qos_profile_default);

    rclc_publisher_init(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/ugv/imu",
        &rmw_qos_profile_default);

    /* -------------------- */
    /* --- Subscription --- */
    /* -------------------- */

    rclc_subscription_init(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        ROS::TOPIC_CMD_VEL,
        &rmw_qos_profile_default);

    rclc_subscription_init(
        &motor_enable_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        ROS::TOPIC_MOTOR_ENABLE,
        &rmw_qos_profile_default);

    rclc_subscription_init(
        &pid_config_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/ugv/pid_config",
        &rmw_qos_profile_default);

    // Timer configuration (from config.h)
    ret = rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(ROS::TIMER_PERIOD_MS), timer_callback);

    // Executor configuration (from config.h)
    unsigned int num_handles = ROS::NUM_HANDLES + 1;
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &motor_enable_sub, &motor_enable_msg, &motor_enable_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &pid_config_sub, &pid_config_msg, &pid_config_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    return true;
}

void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_ret_t ret = RCL_RET_OK;

    ret = rcl_timer_fini(&timer);
    ret = rclc_executor_fini(&executor);
    ret = rcl_init_options_fini(&init_options);
    ret = rcl_node_fini(&node);
    rclc_support_fini(&support);

    ret = rcl_publisher_fini(&odom_pub, &node);
    ret = rcl_publisher_fini(&encoder_pub, &node);
    ret = rcl_publisher_fini(&imu_pub, &node);

    ret = rcl_subscription_fini(&cmd_vel_sub, &node);
    ret = rcl_subscription_fini(&motor_enable_sub, &node);
    ret = rcl_subscription_fini(&pid_config_sub, &node);
}

/**
 * @brief Check if cmd_vel has timed out
 * @return true if no cmd_vel received within Safety::CMD_TIMEOUT_MS
 *
 * Safety feature: Returns true if it's been too long since last cmd_vel,
 * indicating the robot should stop for safety.
 */
bool is_cmd_vel_timeout() {
    // If we've never received a command, no timeout
    if (last_cmd_vel_time_ms == 0) {
        return false;
    }

    // Check if time since last command exceeds timeout threshold
    unsigned long time_since_last_cmd = millis() - last_cmd_vel_time_ms;
    return (time_since_last_cmd > Safety::CMD_TIMEOUT_MS);
}

void ros_loop() {
    prev_state = state;
    switch (state) {
        case WAITING_AGENT:
            state_connected = false;
            EXECUTE_EVERY_N_MS(ROS::AGENT_WAIT_INTERVAL_MS,
                state = (RMW_RET_OK == rmw_uros_ping_agent(ROS::PING_TIMEOUT_MS, ROS::PING_ATTEMPTS)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                state_connected = false;
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(ROS::AGENT_CHECK_INTERVAL_MS,
                state = (RMW_RET_OK == rmw_uros_ping_agent(ROS::PING_TIMEOUT_MS, ROS::PING_ATTEMPTS_CONNECTED)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                state_connected = true;
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS::EXECUTOR_SPIN_TIMEOUT_MS));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state_connected = false;
            // Stop motors on disconnect
            cmd_linear_x = 0.0;
            cmd_angular_z = 0.0;
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}
