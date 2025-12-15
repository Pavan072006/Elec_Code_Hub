/*
Left Front
    -   PWM:    7
    -   DIR:    15
    -   ENCA:   20
    -   ENCB:   19


Right Front
    -   PWM:    16
    -   DIR:    17
    -   ENCA:   13
    -   ENCB:   14

Left Back
    -   PWM:    18
    -   DIR:    08
    -   ENCA:   47
    -   ENCB:   21

Right Back
    -   PWM:    9
    -   DIR:    10
    -   ENCA:   11
    -   ENCB:   12
*/

/*  
    1. Include 4 encoder calibration values - Done
    2. Constrain steer angles from -90 to +90 - Both for individual and PID - Done
    3. Implement I in PID but with ki = 0.00 for now - Done
    4. If you somehow drift, you can go back to -90 to 90 range but you can't further deviate - Done
*/

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

// PCNT - Encoders
#include "driver/pcnt.h"

#define NODE_NAME "drive_node"
#define DOMAIN_ID 0

#define NUM_MOTORS 8
#define NUM_ENC 4
#define NUM_STEER 4
#define PUBLISH_HZ 300       // 300 Hz echo
#define DRIVE_THRESHOLD 185  // for 18v
#define STR_THRESHOLD 128    // for 12v
#define MIN_STEER_PWM 30
#define MAX_ANGLE 100

// static const float ENC_Calib = 8.301f / 1000;
static const float ENC_Calib_Array[NUM_ENC] = {
    8.301f / 1000,  // FL 
    8.301f / 1000,  // FR 
    (360.0f / 95940) * 5.7,  // BL 
    8.301f / 1000   // BR 
};
// int desired_angle_buffer[NUM_ENC] = {0};     // desired angles buffer
int steer_pwm[NUM_ENC] = { 0 };

// PCNT
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768

static void IRAM_ATTR pcnt_intr_handler(void *arg);
void PCNT_Init(pcnt_unit_t unit_num);

int PWMpin[NUM_MOTORS] = { 40, 38, 36, 48, 7, 16, 18, 9 };   // front two, followed by back two, LRLR    // Steering- FL, FR, BL, BR
int dirpin[NUM_MOTORS] = { 41, 39, 37, 35, 15, 17, 8, 10 };  // first four are drive.                    // Steering- FL, FR, BL, BR
int defaultdir[NUM_MOTORS] = { 1, 1, 1, 1, 1, 0, 0, 0 };     // change only when hardware changes        // Steering- FL, FR, BL, BR
int enc_dir[NUM_ENC] = { -1, -1, -1, -1 };                   // Steering- FL, FR, BL, BR

int32_t debug[12] = { 0 };  // [0..3]=desired angles, [4..7]=enc, [8..11]=steer_pwm

// int encA[NUM_ENC] = {11,13,47,20,4,1};
// int encB[NUM_ENC] = {12,14,21,19,5,2};
int encA[NUM_ENC] = { 20, 13, 47, 11 };  // Steering- FL, FR, BL, BR
int encB[NUM_ENC] = { 19, 14, 21, 12 };  // Steering- FL, FR, BL, BR
float enc_buf[NUM_ENC] = { 0 };
float temp_enc_buf[NUM_ENC] = { 0 };
volatile int accumulator[NUM_ENC] = { 0 };  // only 4 PCNT units used
int pwm[4] = { 0 };

// pid
const float kp[NUM_STEER] = { 5.0f, 5.0f, 5.0f, 5.0f };
const float ki[NUM_STEER] = { 0.00f, 0.00f, 0.00f, 0.00f };
const int max_integral = 500;
float integral_error[NUM_STEER] = { 0.0f };

bool isInIndividualSteeringBuffer = false;
bool lastIndSteer = false;

rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
rcl_subscription_t indsteer_sub;
// rcl_subscription_t desired_angles_subscriber;
rcl_publisher_t publisher;
rcl_publisher_t publisher_enc;
rcl_publisher_t publisher_debug;  // NEW: debug publisher

rcl_timer_t timer;

std_msgs__msg__Int32MultiArray pwm_msg;
std_msgs__msg__Float32MultiArray enc_auto_msg;
std_msgs__msg__Int32MultiArray debug_msg;  // NEW: debug message
std_msgs__msg__Bool indsteer_msg;
// std_msgs__msg_Float32MultiArray desired_angles_msg;

int32_t drive_buf[NUM_MOTORS] = { 0 };

// Steering defaults
int max_manual_angle = 180;  // degrees

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) { delay(100); }
}

void angle_to_pwm(const int32_t *drive_buf, const float *enc_buf) {
  int deadband = 1;

  for (int i = 0; i < NUM_STEER; i++) {
    int calculated_pwm = 0; // Initialize to 0 just to be safe
    
    int32_t desired_angle = drive_buf[i + 4];
    
    // Calculate PID
    int error = (int)(desired_angle - enc_buf[i]);

    if (error > deadband || error < -deadband) {
      integral_error[i] += error;

      if (integral_error[i] > max_integral) integral_error[i] = max_integral;
      if (integral_error[i] < -max_integral) integral_error[i] = -max_integral;

      int p_term = (int) (error * kp[i]);
      int i_term = (int) (integral_error[i] * ki[i]);

      calculated_pwm = p_term + i_term; 
    } else {
      calculated_pwm = 0;
      integral_error[i] = 0;
    }

    if (calculated_pwm > -MIN_STEER_PWM && calculated_pwm < MIN_STEER_PWM) {
      calculated_pwm = 0;
    }

    // 4. Assign Final Value
    steer_pwm[i] = calculated_pwm;
  }
}

// Callback for subscriber
void pwm_sub_callback(const void *msg) {
  const std_msgs__msg__Int32MultiArray *incoming = (const std_msgs__msg__Int32MultiArray *)msg;

  int sz = incoming->data.size < NUM_MOTORS ? incoming->data.size : NUM_MOTORS;

  for (int i = 0; i < sz; i++) {
    drive_buf[i] = incoming->data.data[i];
  }
}

void indsteer_sub_callback(const void *msg) {
  const std_msgs__msg__Bool *incoming = (const std_msgs__msg__Bool *)msg;
  isInIndividualSteeringBuffer = incoming->data;
}

// Timer callback to publish echo + encoders + debug
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;

  // motor_pwm_echo
  pwm_msg.data.size = NUM_MOTORS;
  pwm_msg.data.capacity = NUM_MOTORS;
  rcl_publish(&publisher, &pwm_msg, NULL);

  // enc_auto
  enc_auto_msg.data.data = temp_enc_buf;
  enc_auto_msg.data.size = NUM_ENC;
  enc_auto_msg.data.capacity = NUM_ENC;
  rcl_publish(&publisher_enc, &enc_auto_msg, NULL);

  // debug
  debug_msg.data.size = 12;
  debug_msg.data.capacity = 12;
  rcl_publish(&publisher_debug, &debug_msg, NULL);
}

void setup() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PWMpin[i], OUTPUT);
    pinMode(dirpin[i], OUTPUT);
  }

  // install PCNT ISR service
  pcnt_isr_service_install(0);

  for (int i = 0; i < 4; i++) {
    PCNT_Init((pcnt_unit_t)i);
  }

  Serial.begin(921600);
  delay(1000);
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  RCUTILS_LOG_INFO_NAMED(NODE_NAME, "Started with domain id: %d", DOMAIN_ID);

  // Subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "motor_pwm");

  rclc_subscription_init_default(
    &indsteer_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "IndSteer");

  // Publishers
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "motor_pwm_echo");

  rclc_publisher_init_default(
    &publisher_enc,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "enc_auto");

  // NEW: debug publisher
  rclc_publisher_init_default(
    &publisher_debug,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "debug");

  // Attach buffers
  pwm_msg.data.data = drive_buf;
  pwm_msg.data.size = 0;
  pwm_msg.data.capacity = NUM_MOTORS;

  enc_auto_msg.data.data = enc_buf;
  enc_auto_msg.data.size = 0;
  enc_auto_msg.data.capacity = NUM_ENC;

  debug_msg.data.data = debug;  // NEW: bind debug array
  debug_msg.data.size = 0;
  debug_msg.data.capacity = 12;

  // Timer at PUBLISH_HZ
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000 / PUBLISH_HZ),
    timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &pwm_msg, pwm_sub_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &indsteer_sub, &indsteer_msg, indsteer_sub_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  RCUTILS_LOG_INFO_NAMED(NODE_NAME, "Setup Complete");
}

void loop() {
  rclc_executor_spin_some(&executor, 0);
  if (isInIndividualSteeringBuffer) {
    for (int i = 0; i < 4; i++) {
      enc_buf[i] = 0.0;  
    }
  } 
  else {
    if (lastIndSteer) {
      for (int i = 0; i < 4; i++) {
        accumulator[i] = 0;
        pcnt_counter_clear((pcnt_unit_t)i);
      }
    }
  }

  // Drive Motors (first 4)
  for (int i = 0; i < 4; i++) {
    if (drive_buf[i] >= 0) {
      analogWrite(PWMpin[i], min(DRIVE_THRESHOLD, (int)drive_buf[i]));
      digitalWrite(dirpin[i], defaultdir[i]);
    } else {
      analogWrite(PWMpin[i], min(DRIVE_THRESHOLD, (int)-drive_buf[i]));
      digitalWrite(dirpin[i], 1 - defaultdir[i]);
    }
  }
  if (isInIndividualSteeringBuffer) {
    for (int i = 4; i < 8; i++) 
    {
      bool safe_to_move = true;
      float current_angle = temp_enc_buf[i-4];
      if (current_angle > MAX_ANGLE && drive_buf[i] > 0) safe_to_move = false; // Don't go further positive
      if (current_angle < -MAX_ANGLE && drive_buf[i] < 0) safe_to_move = false; // Don't go further negative
      if (safe_to_move) 
      {
        if (drive_buf[i] >= 0)
        {
          analogWrite(PWMpin[i], (min(STR_THRESHOLD, (int)drive_buf[i])));
          digitalWrite(dirpin[i], defaultdir[i]);
        } 
        else 
        {
          analogWrite(PWMpin[i], (min(STR_THRESHOLD, (int)-drive_buf[i])));
          digitalWrite(dirpin[i], 1 - defaultdir[i]);
        }
      }
      else {analogWrite(PWMpin[i], 0);}
    }
  } 
  else 
  {
    // Steering Motors (last 4)
    angle_to_pwm(drive_buf, enc_buf);
    for (int i = 0; i < 4; i++) 
    {
      bool safe_to_move = true;
      float current_angle = temp_enc_buf[i];
      if (current_angle > MAX_ANGLE && steer_pwm[i] > 0) safe_to_move = false; // Don't go further positive
      if (current_angle < -MAX_ANGLE && steer_pwm[i] < 0) safe_to_move = false; // Don't go further negative
      if (safe_to_move) 
      {
        if (steer_pwm[i] >= 0)
        {
          analogWrite(PWMpin[i + 4], (min(STR_THRESHOLD, (int)steer_pwm[i])));
          digitalWrite(dirpin[i + 4], defaultdir[i + 4]);
        } 
        else 
        {
          analogWrite(PWMpin[i + 4], (min(STR_THRESHOLD, (int)-steer_pwm[i])));
          digitalWrite(dirpin[i + 4], 1 - defaultdir[i + 4]);
        }
      }
      else {analogWrite(PWMpin[i + 4], 0);}
    }
  }

  // Read encoders into enc_buf
  for (int i = 0; i < 4; i++) {
    int16_t count = 0;
    pcnt_get_counter_value((pcnt_unit_t)i, &count);
    temp_enc_buf[i] = (accumulator[i] * (int64_t)PCNT_H_LIM + (int64_t)count) * ENC_Calib_Array[i] * enc_dir[i];
    enc_buf[i] = temp_enc_buf[i];
  }

  // Fill debug array:
  // [0..3]   = desired angles (drive_buf[4..7])
  // [4..7]   = encoder values (enc_buf[0..3]) cast to int
  // [8..11]  = steer_pwm[0..3]
  for (int i = 0; i < 4; i++) {
    debug[i] = (int)drive_buf[i + 4];
  }
  for (int i = 4; i < 8; i++) {
    debug[i] = (int)enc_buf[i - 4];
  }
  for (int i = 8; i < 12; i++) {
    debug[i] = (int)steer_pwm[i - 8];
  }
  lastIndSteer = isInIndividualSteeringBuffer;
}

// ---- PCNT init and ISR ----
void PCNT_Init(pcnt_unit_t unit_num) {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = encA[unit_num],
    .ctrl_gpio_num = encB[unit_num],
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DEC,
    .counter_h_lim = PCNT_H_LIM,
    .counter_l_lim = PCNT_L_LIM,
    .unit = unit_num,
    .channel = PCNT_CHANNEL_0
  };

  pcnt_unit_config(&pcnt_config);

  pcnt_event_enable(unit_num, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit_num, PCNT_EVT_L_LIM);

  pcnt_isr_handler_add(unit_num, pcnt_intr_handler, (void *)(int)unit_num);

  pcnt_counter_pause(unit_num);
  pcnt_counter_clear(unit_num);
  pcnt_counter_resume(unit_num);
}

static void IRAM_ATTR pcnt_intr_handler(void *arg) {
  int unit_val = (int)arg;
  pcnt_unit_t unit = (pcnt_unit_t)unit_val;

  uint32_t evt_status = 0;
  pcnt_get_event_status(unit, &evt_status);

  if (evt_status & PCNT_EVT_H_LIM) {
    accumulator[unit_val]++;
  }
  if (evt_status & PCNT_EVT_L_LIM) {
    accumulator[unit_val]--;
  }
}