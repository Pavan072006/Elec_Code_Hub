#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rcl/subscription.h>
#include <rmw/qos_profiles.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

#define NUM_MOT 6
#define MAX_PWM 255


// Base, Shoulder, Elbow, Bevel 1, Bevel 2, Gripper
int encA_pins[NUM_MOT] = {39, 48, 21, 14, 12, 10};
int encB_pins[NUM_MOT] = {40, 38, 47, 13, 11, 9};
static const int enc_dir[NUM_MOT] = {1, 1, 1, 1, 1, 1};

volatile int last_a[NUM_MOT] = {0};
volatile int last_b[NUM_MOT] = {0};

// Base, Shoulder, Elbow, Bevel 1, Bevel2, Gripper
int pwm_pins[NUM_MOT] =  {2, 41, 6, 4, 15, 17};     
int dir_pins[NUM_MOT] =  {1, 42, 7, 5, 16, 18};
static const int dir_dir[NUM_MOT] = {1, 1, 1, 1, 1, 1};

volatile long enc_count[NUM_MOT] = {0};

static const float enc_to_angle_factor[NUM_MOT] = {
    180.0f / 143082.5f,         // Base
    90.0f / 390668.0f,          // Shoulder
    90.0f / 64735.3f,           // Elbow
    90.0f / 55157.0f,           // Bevel 1
    90.0f / 55157.0f,           // Bevel 2
    90.0f / 55157.0f            // Gripper
};

float current_angle[NUM_MOT];
// Stores either target angles or raw PWM values depending on mode
float target_data[NUM_MOT] = {0};

float Kp[NUM_MOT] = {2,2,2,2,2,2};
float Ki[NUM_MOT] = {0,0,0,0,0,0};
float Kd[NUM_MOT] = {0,0,0,0,0,0};

float integral[NUM_MOT]   = {0};
float prev_error[NUM_MOT] = {0};

// --- Mode flags ---
// /input_space False + /arm_state False  -> all 6 joints: direct PWM
// /input_space True  + /arm_state False  -> joints 0-2: angle PID, joints 3-5: direct PWM
// /arm_state True (any /input_space)     -> all 6 joints: angle PID
volatile bool input_space = false;
volatile bool arm_state   = false;

// Micro-ROS handles
rcl_node_t      node;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;

rcl_subscription_t sub;
std_msgs__msg__Float32MultiArray target_msg;

rcl_subscription_t input_space_sub;
std_msgs__msg__Bool input_space_msg;

rcl_subscription_t arm_state_sub;
std_msgs__msg__Bool arm_state_msg;

rcl_publisher_t enc_publisher;
std_msgs__msg__Int32MultiArray actual_enc_msg;

rcl_publisher_t echo_publisher;
std_msgs__msg__Float32MultiArray target_echo_msg;

rcl_publisher_t angle_publisher;
std_msgs__msg__Float32MultiArray actual_angle_msg;

rcl_timer_t timer;

// ---- Helpers ----

inline bool joint_is_pwm(int i) {
    if (arm_state) return false;           // arm_state True -> all angle
    if (!input_space) return true;         // both False     -> all PWM
    return (i >= 3);                       // mixed          -> 0-2 angle, 3-5 PWM
}

inline void drive_pwm(int i, float raw) {
    int val = (int)raw;
    int pwm = constrain(abs(val), 0, MAX_PWM);
    int dir = (val >= 0) ? HIGH : LOW;
    digitalWrite(dir_pins[i], dir);
    analogWrite(pwm_pins[i], pwm);
}

// ---- Encoders ----

inline void updateEncoder(int i) {
    int a = digitalRead(encA_pins[i]);
    int b = digitalRead(encB_pins[i]);

    int last_a_state = last_a[i];
    int last_b_state = last_b[i];

    if (a != last_a_state || b != last_b_state) {
        if (last_a_state == a) {
            enc_count[i] += (a ^ b) ? -1 : +1;
        } else {
            enc_count[i] += (a ^ b) ? +1 : -1;
        }
        last_a[i] = a;
        last_b[i] = b;
    }
}

void IRAM_ATTR isr0A() {updateEncoder(0);}
void IRAM_ATTR isr0B() {updateEncoder(0);}
void IRAM_ATTR isr1A() {updateEncoder(1);}
void IRAM_ATTR isr1B() {updateEncoder(1);}
void IRAM_ATTR isr2A() {updateEncoder(2);}
void IRAM_ATTR isr2B() {updateEncoder(2);}
void IRAM_ATTR isr3A() {updateEncoder(3);}
void IRAM_ATTR isr3B() {updateEncoder(3);}
void IRAM_ATTR isr4A() {updateEncoder(4);}
void IRAM_ATTR isr4B() {updateEncoder(4);}
void IRAM_ATTR isr5A() {updateEncoder(5);}
void IRAM_ATTR isr5B() {updateEncoder(5);}

// ---- Callbacks ----

void target_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    int n = msg->data.size < NUM_MOT ? msg->data.size : NUM_MOT;
    for (int i = 0; i < n; i++) {
        target_data[i] = msg->data.data[i];
    }
    // Reset PID state for joints switching to angle control
    // so stale integral/derivative don't cause a jerk
    for (int i = 0; i < NUM_MOT; i++) {
        if (!joint_is_pwm(i)) {
            integral[i]   = 0;
            prev_error[i] = 0;
        }
    }
}

void input_space_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    input_space = msg->data;
}

void arm_state_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    arm_state = msg->data;
}

// ---- PID + Drive ----

void run_pid() {
    for (int i = 0; i < NUM_MOT; i++) {
        if (joint_is_pwm(i)) {
            drive_pwm(i, target_data[i]);
        } else {
            noInterrupts();
            long count = enc_count[i];
            interrupts();

            current_angle[i] = count * enc_to_angle_factor[i];
            float error      = target_data[i] - current_angle[i];

            integral[i] += error * 0.02f;
            integral[i]  = constrain(integral[i], -1000, 1000);

            float derivative = (error - prev_error[i]) / 0.02f;
            float output     = Kp[i] * error + Ki[i] * integral[i] + Kd[i] * derivative;

            prev_error[i] = error;

            int pwm = constrain((int)abs(output), 0, MAX_PWM);
            int dir = (output >= 0) ? HIGH : LOW;
            digitalWrite(dir_pins[i], dir);
            analogWrite(pwm_pins[i], pwm);
        }
    }
}

// ---- Timer ----

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    run_pid();

    for (int i = 0; i < NUM_MOT; i++) {
        noInterrupts();
        long count = enc_count[i];
        interrupts();

        float angle = count * enc_to_angle_factor[i];
        current_angle[i] = angle;

        target_echo_msg.data.data[i]  = target_data[i];
        actual_enc_msg.data.data[i]   = (int32_t)count;
        actual_angle_msg.data.data[i] = angle;
    }

    rcl_publish(&echo_publisher, &target_echo_msg, NULL);
    rcl_publish(&enc_publisher,  &actual_enc_msg,  NULL);
    rcl_publish(&angle_publisher, &actual_angle_msg, NULL);
}

// ---- Encoder setup ----

void setupEncoders() {
    for (int i = 0; i < NUM_MOT; i++) {
        pinMode(pwm_pins[i], OUTPUT);
        pinMode(dir_pins[i], OUTPUT);
    }
    for (int i = 0; i < NUM_MOT; i++) {
        pinMode(encA_pins[i], INPUT_PULLUP);
        pinMode(encB_pins[i], INPUT_PULLUP);
        last_a[i] = digitalRead(encA_pins[i]);
        last_b[i] = digitalRead(encB_pins[i]);
    }
    attachInterrupt(digitalPinToInterrupt(encA_pins[0]), isr0A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[0]), isr0B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[1]), isr1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[1]), isr1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[2]), isr2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[2]), isr2B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[3]), isr3A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[3]), isr3B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[4]), isr4A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[4]), isr4B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[5]), isr5A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[5]), isr5B, CHANGE);
}

// ---- Setup / Loop ----

void setup() {
    for (int i = 0; i < NUM_MOT; i++) {
        pinMode(pwm_pins[i], OUTPUT);
        pinMode(dir_pins[i], OUTPUT);
        analogWrite(pwm_pins[i], 0);
        digitalWrite(dir_pins[i], dir_dir[i]);
    }

    setupEncoders();
    set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "arm_node", "", &support);

    // Subscribers
    rclc_subscription_init_default(
        &sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/arm_target_angles"
    );
    rclc_subscription_init_default(
        &input_space_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/input_space"
    );
    rclc_subscription_init_default(
        &arm_state_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/arm_state"
    );

    // Message buffers
    target_echo_msg.data.data     = (float*)   malloc(NUM_MOT * sizeof(float));
    target_echo_msg.data.size     = NUM_MOT;
    target_echo_msg.data.capacity = NUM_MOT;

    target_msg.data.data     = (float*)   malloc(NUM_MOT * sizeof(float));
    target_msg.data.size     = NUM_MOT;
    target_msg.data.capacity = NUM_MOT;

    actual_enc_msg.data.data     = (int32_t*) malloc(NUM_MOT * sizeof(int32_t));
    actual_enc_msg.data.size     = NUM_MOT;
    actual_enc_msg.data.capacity = NUM_MOT;

    actual_angle_msg.data.data     = (float*)   malloc(NUM_MOT * sizeof(float));
    actual_angle_msg.data.size     = NUM_MOT;
    actual_angle_msg.data.capacity = NUM_MOT;

    // Publishers
    rclc_publisher_init_default(
        &echo_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/arm_target_echo"
    );
    rclc_publisher_init_default(
        &enc_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/enc_counts"
    );
    rclc_publisher_init_default(
        &angle_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/enc_angles"
    );

    // Timer
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback);

    // Executor — 4 handles: 3 subs + 1 timer
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_subscription(&executor, &sub,            &target_msg,      &target_callback,      ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &input_space_sub, &input_space_msg, &input_space_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &arm_state_sub,   &arm_state_msg,   &arm_state_callback,   ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
