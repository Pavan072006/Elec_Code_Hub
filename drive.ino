/*

Written by: Adeesh
Date: 23 May, 2026

Description:
micro-ROS based 6-motor drive controller for ESP32-S3 with indicator

Features:
- Subscribes to:
    /drive_pwm   -> std_msgs/msg/Int32MultiArray
    /state       -> std_msgs/msg/Bool
- Publishes:
    /drive_pwm_echo
    /state_echo
- Controls 6 motors using PWM + direction pins
- Watchdog safety stop if /drive_pwm messages are not received
- Transport failure detection with automatic ESP restart
- Relay/indicator blinking support
- RGB status indication:
    Blue   -> Waiting for micro-ROS agent
    Green  -> Normal operation
    Yellow -> Watchdog triggered (no drive messages)
    Red    -> Transport disconnected, restarting ESP

Hardware Mapping:

Motor         PWM Pin     DIR Pin
---------------------------------
1             36          37
2             38          39
3             40          41
4             42          2
5             4           5
6             6           7

Indicator Outputs:
- GPIO 48 -> Red Indicator

Notes:
- PWM input range: -127 to 127
- Negative values reverse motor direction
- Motors stop automatically on watchdog timeout
- Designed for ROS 2 + micro-ROS serial transport

*/

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>
#include <rmw/qos_profiles.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>

#define NODE_NAME "drive_node"
#define DOMAIN_ID 0
#define NUM_MOTORS 6
#define MAX_PWM_VALUE 127
#define TIMER_PERIOD_MS 1000
#define WATCHDOG_TRIGGER_TIME_IN_S 1
// #define INDICATOR_RED_GPIO 35
#define INDICATOR_RED_GPIO 48
#define RELAY_BLINK_MS 1000

static const int PWMpins[NUM_MOTORS] = {36,38,40,42,4,6};
static const int DIRpins[NUM_MOTORS] = {37,39,41,2,5,7};
static const int motor_directions[NUM_MOTORS] = {0,0,0,0,0,0};

rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;

rcl_timer_t timer;
rcl_subscription_t pwm_subscriber;
std_msgs__msg__Int32MultiArray pwm_msg;
int32_t drive_buffer[NUM_MOTORS] = {0};
rcl_subscription_t state_subscriber;  // Relay state sub
std_msgs__msg__Bool state_msg;
bool relay_state_buffer = 0;
rcl_publisher_t pwm_echo_publisher;
std_msgs__msg__Int32MultiArray pwm_echo_msg;
int32_t echo_drive_buffer[NUM_MOTORS] = {0};
rcl_publisher_t state_echo_publisher;
std_msgs__msg__Bool state_echo_msg;
rcl_publisher_t gpio_state_publisher;
std_msgs__msg__Bool gpio_state_msg;
uint32_t last_msg_time = 0;
uint32_t last_blink_time = 0;
bool relay_blink_state = false;

static int transport_fail_count = 0;

#define TRANSPORT_RCCHECK(fn) { \
  rcl_ret_t rc = fn; \
  if (rc!=RCL_RET_OK) { \
    transport_error_loop(); \
  } \
}

#define RUNTIME_RCCHECK(fn) { \
  rcl_ret_t rc = fn; \
  if (rc!=RCL_RET_OK) { \
    runtime_error_loop(); \
  } \
}

void transport_error_loop() {
  for (int i=0; i<NUM_MOTORS; i++){
    analogWrite(PWMpins[i], 0);
  }
  digitalWrite(INDICATOR_RED_GPIO, 0);
  // digitalWrite(INDICATOR_GREEN_GPIO, 0);
  state_echo_msg.data = false;

  while (1) {
    neopixelWrite(RGB_BUILTIN, 150, 0, 0);
    delay(1000);

    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
      ESP.restart();
    }
  }
}

void runtime_error_loop() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    analogWrite(PWMpins[i], 0);
  }
  while (1) {
    neopixelWrite(RGB_BUILTIN, 0, 0, 150);
    delay(100);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(100);
  }
}

void pwm_subscription_callback(const void *msg) {
  last_msg_time = millis();
  const std_msgs__msg__Int32MultiArray *in = (const std_msgs__msg__Int32MultiArray *)msg;
  int size = (in->data.size < NUM_MOTORS) ? in->data.size : NUM_MOTORS;
  for (int i=0; i < NUM_MOTORS; i++) {
    if (i < size) {
      drive_buffer[i] = in->data.data[i];
    }
    else {
      drive_buffer[i] = 0;
    }
  }
}

void state_subscription_callback(const void *msg) {
  const std_msgs__msg__Bool *in = (const std_msgs__msg__Bool *)msg;
  relay_state_buffer = in->data;
  // last_msg_time = millis();e
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;

  pwm_echo_msg.data.data = echo_drive_buffer;
  pwm_echo_msg.data.size = NUM_MOTORS;
  pwm_echo_msg.data.capacity = NUM_MOTORS;
  if(rcl_publish(&pwm_echo_publisher, &pwm_echo_msg, NULL)!=RCL_RET_OK) {
    transport_error_loop();
  }
  state_echo_msg.data = relay_state_buffer;
  // state_echo_msg.data = relay_state_buffer;
  if(rcl_publish(&state_echo_publisher, &state_echo_msg, NULL)!=RCL_RET_OK) {
    transport_error_loop();
  }
  if (rcl_publish(&gpio_state_publisher, &gpio_state_msg, NULL) != RCL_RET_OK) {
    transport_error_loop();
  }
}

void setup() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PWMpins[i], OUTPUT);
    pinMode(DIRpins[i], OUTPUT);
    analogWrite(PWMpins[i], 0);
    digitalWrite(DIRpins[i], 0);
  }
  pinMode(INDICATOR_RED_GPIO, OUTPUT);
  digitalWrite(INDICATOR_RED_GPIO, 0);
  // pinMode(INDICATOR_GREEN_GPIO, OUTPUT);
  // digitalWrite(INDICATOR_GREEN_GPIO, 0);
  state_echo_msg.data = false;
  delay(1000);
  set_microros_transports();

  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    neopixelWrite(RGB_BUILTIN, 0, 0, 150);
    delay(100);
  }

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  RUNTIME_RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));
  RUNTIME_RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RUNTIME_RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  RUNTIME_RCCHECK(rclc_subscription_init_default(&pwm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/drive_pwm"));
  pwm_msg.data.data = drive_buffer;
  pwm_msg.data.capacity = NUM_MOTORS;
  pwm_msg.data.size = 0;

  RUNTIME_RCCHECK(rclc_subscription_init_default(&state_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/state"));

  RUNTIME_RCCHECK(rclc_publisher_init_default(&pwm_echo_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/drive_pwm_echo"));
  pwm_echo_msg.data.data = echo_drive_buffer;
  RUNTIME_RCCHECK(
    rclc_publisher_init_default(
      &gpio_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/gpio_state"
    )
  );
  RUNTIME_RCCHECK(rclc_publisher_init_default(&state_echo_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/state_echo"));
  // state_echo_msg.data = relay_state_buffer;

  // RUNTIME_RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(TIMER_PERIOD_MS), timer_callback));
  RUNTIME_RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(TIMER_PERIOD_MS), timer_callback));

  RUNTIME_RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RUNTIME_RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, pwm_subscription_callback, ON_NEW_DATA));
  RUNTIME_RCCHECK(rclc_executor_add_subscription(&executor, &state_subscriber, &state_msg, state_subscription_callback, ON_NEW_DATA));
  RUNTIME_RCCHECK(rclc_executor_add_timer(&executor, &timer));

  last_msg_time = millis();
}

void loop() {
  if(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)) != RCL_RET_OK) {
    transport_fail_count++;
    if (transport_fail_count > 5){
      transport_error_loop();
    }
  }
  else {
    transport_fail_count = 0;
  }

  for(int i=0; i<NUM_MOTORS; i++) {
    echo_drive_buffer[i] = drive_buffer[i];
  }
  // pwm_echo_msg.data.data = echo_drive_buffer;
  
  if (millis() - last_msg_time < (WATCHDOG_TRIGGER_TIME_IN_S*1000)){  // Normal Operation
    for (int i = 0; i < NUM_MOTORS; i++) {
      int pwm_input = constrain(abs(drive_buffer[i]), 0, MAX_PWM_VALUE);
      analogWrite(PWMpins[i], pwm_input);
      digitalWrite(DIRpins[i], drive_buffer[i] >= 0 ? motor_directions[i] : !motor_directions[i]);
    }
    if (millis() - last_blink_time >= RELAY_BLINK_MS) {
      last_blink_time = millis();
      relay_blink_state = !relay_blink_state;
      // if (relay_state_buffer) {
      //   // Green Indicator
      //   digitalWrite(INDICATOR_RED_GPIO, 0);
      //   digitalWrite(INDICATOR_GREEN_GPIO, relay_blink_state);
      // }
      // else {
      //   digitalWrite(INDICATOR_RED_GPIO, relay_blink_state);
      //   digitalWrite(INDICATOR_GREEN_GPIO, 0);
      // }
      digitalWrite(INDICATOR_RED_GPIO, relay_blink_state);
      gpio_state_msg.data = relay_blink_state;
    }
    neopixelWrite(RGB_BUILTIN, 0, 100, 0);
  }
  else {    // Watchdog
    for (int i = 0; i < NUM_MOTORS; i++) {
      int pwm_input = 0;
      analogWrite(PWMpins[i], pwm_input);
      digitalWrite(DIRpins[i], motor_directions[i]);
    }
    relay_blink_state = false;
    digitalWrite(INDICATOR_RED_GPIO, 1);
    gpio_state_msg.data = true;
    // digitalWrite(INDICATOR_GREEN_GPIO, 0);
    last_blink_time = millis();
    neopixelWrite(RGB_BUILTIN, 150, 150, 0);
  }
}