#include "ros_com.h"
#include "encoder_handler.h"
#include "kinematics.h"

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

// Subscribers
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t motor_enable_sub;

/* ------------------------ */
/* --- ROS Msg Declare --- */
/* ----------------------- */
// Publisher messages
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32MultiArray encoder_msg;

// Subscriber messages
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Bool motor_enable_msg;

/* -------------------- */
/* --- ROS Msg init --- */
/* -------------------- */
void init_ros_msgs() {

    // Initialize odometry message
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
    odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

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

    // Initialize encoder message (2 encoders: left, right)
    static int32_t encoder_data[2] = {0, 0};
    encoder_msg.data.capacity = 2;
    encoder_msg.data.size = 2;
    encoder_msg.data.data = encoder_data;

    // Initialize subscriber messages
    motor_enable_msg.data = true;

    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;
}

/* ------------------------ */
/* --- ROS Sub Callback --- */
/* ------------------------ */
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    cmd_linear_x = msg->linear.x;
    cmd_angular_z = msg->angular.z;
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

/* -------------------------- */
/* --- ROS Timer Callback --- */
/* -------------------------- */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {

        rcl_ret_t ret = RCL_RET_OK;

        // Get real encoder counts from hardware
        int32_t current_left, current_right;
        getEncoderCounts(current_left, current_right);

        // Calculate encoder deltas since last update
        int32_t delta_left = current_left - last_encoder_left;
        int32_t delta_right = current_right - last_encoder_right;

        // Update odometry using real encoder data and kinematics
        updateOdometry(delta_left, delta_right, odom_x, odom_y, odom_theta);

        // Save current encoder counts for next iteration
        last_encoder_left = current_left;
        last_encoder_right = current_right;

        // Get current wheel speeds for twist velocity
        float left_speed, right_speed;
        getWheelSpeeds(left_speed, right_speed);

        // Convert wheel speeds to twist (linear_x, angular_z)
        float linear_x, angular_z;
        wheelSpeedsToTwist(left_speed, right_speed, linear_x, angular_z);

        // Publish Odometry with real data
        odom_msg.header.stamp.sec = (int32_t)(millis() / 1000);
        odom_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

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
    }
}

////////////////////////////////////
/// ROS Init & Destroy Functions ///
////////////////////////////////////
bool create_entities() {

    const char * node_name = "general_driver";
    const char * ns = "";
    const int domain_id = 0;

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

    rclc_publisher_init_best_effort(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/ugv/odom");

    rclc_publisher_init_best_effort(
        &encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/ugv/encoder");

    /* -------------------- */
    /* --- Subscription --- */
    /* -------------------- */

    rclc_subscription_init_best_effort(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    rclc_subscription_init_best_effort(
        &motor_enable_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/ugv/motor_enable");

    // Timer for publishing at 20Hz (50ms)
    const unsigned int timer_timeout = 50;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

    // num_handles = subscribers + timer
    unsigned int num_handles = 3; // 2 subscribers + 1 timer
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &motor_enable_sub, &motor_enable_msg, &motor_enable_callback, ON_NEW_DATA);
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

    ret = rcl_subscription_fini(&cmd_vel_sub, &node);
    ret = rcl_subscription_fini(&motor_enable_sub, &node);
}

void ros_loop() {
    prev_state = state;
    switch (state) {
        case WAITING_AGENT:
            state_connected = false;
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                state_connected = false;
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                state_connected = true;
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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
