#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/subscription.h>
#include <rmw/qos_profiles.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#define NUM_ENC 3
#define NUM_MOTORS 6
#define MAX_PWM_VALUE 255
#define TIMER_PERIOD_MS 10  // 40Hz for better control response

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { Serial.print(#fn " failed: "); Serial.println(rc); error_loop(); } }
#define RCSOFTCHECK(fn) { (void)fn; }

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray pwm_int;  

rcl_publisher_t publisher;       // encoder publisher (Float32MultiArray)
rcl_publisher_t echo_publisher;  // republish incoming /stm_write to /stm_write_echo           

std_msgs__msg__Float32MultiArray enc_msg;  
std_msgs__msg__Int32MultiArray pwm_echo;     

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Encoder variables with interrupt protection
volatile long enc_pos[NUM_ENC] = {0};
volatile bool A_set[NUM_ENC] = {false};
volatile bool B_set[NUM_ENC] = {false};
volatile bool enc_updated = false;

// Motor pin configurations (keep your original pins)
int PWMpin[NUM_MOTORS] = {4, 6, 16, 18, 9, 11};
int dirpin[NUM_MOTORS] = {5, 7, 15, 17, 8, 10};
int defaultdir[NUM_MOTORS] = {1, 1, 1, 1, 1, 1};

// ESP32-S3 encoder pins
int encB[NUM_ENC] = {42, 38, 47};
int encA[NUM_ENC] = {41, 37, 21};

// Data buffers — single backing array pattern (like your working drive code)
int32_t drive_buf[NUM_MOTORS] = {0};      // single static buffer used for subscriber and echo
float enc_feed[NUM_ENC] = {0};

// Conversion factor - adjust based on your encoder specifications
const float ENC_TO_DEGREES = 90.0 / 55157.0;

// Safety limits
const int MIN_PWM = -MAX_PWM_VALUE;
const int MAX_PWM = MAX_PWM_VALUE;

// --- Encoder ISRs (unchanged except small tidy) ---
void IRAM_ATTR callback_0() {
  static unsigned long last_interrupt_0 = 0;
  unsigned long interrupt_time = micros();
  if (interrupt_time - last_interrupt_0 > 100) {
    int i = 0;
    bool A_current = digitalRead(encA[i]);
    bool B_current = digitalRead(encB[i]);
    if (A_current != A_set[i]) {
      A_set[i] = A_current;
      if (A_current == B_current) enc_pos[i]++;
      else enc_pos[i]--;
      enc_updated = true;
    }
  }
  last_interrupt_0 = interrupt_time;
}

void IRAM_ATTR callback_1() {
  static unsigned long last_interrupt_1 = 0;
  unsigned long interrupt_time = micros();
  if (interrupt_time - last_interrupt_1 > 100) {
    int i = 1;
    bool A_current = digitalRead(encA[i]);
    bool B_current = digitalRead(encB[i]);
    if (A_current != A_set[i]) {
      A_set[i] = A_current;
      if (A_current == B_current) enc_pos[i]++;
      else enc_pos[i]--;
      enc_updated = true;
    }
  }
  last_interrupt_1 = interrupt_time;
}

void IRAM_ATTR callback_2() {
  static unsigned long last_interrupt_2 = 0;
  unsigned long interrupt_time = micros();
  if (interrupt_time - last_interrupt_2 > 100) {
    int i = 2;
    bool A_current = digitalRead(encA[i]);
    bool B_current = digitalRead(encB[i]);
    if (A_current != A_set[i]) {
      A_set[i] = A_current;
      if (A_current == B_current) enc_pos[i]++;
      else enc_pos[i]--;
      enc_updated = true;
    }
  }
  last_interrupt_2 = interrupt_time;
}

void error_loop() {
  while (1) { digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(100); }
}

// --- Subscription callback: write directly into shared drive_buf (like drive code) ---
void subscription_callback(const void * msg) {
  const std_msgs__msg__Int32MultiArray *incoming = (const std_msgs__msg__Int32MultiArray *)msg;
  
  int sz = incoming->data.size < NUM_MOTORS ? incoming->data.size : NUM_MOTORS;

  for (int i = 0; i < sz; i++) {
    drive_buf[i] = incoming->data.data[i];
  }

  // If incoming sent fewer values, zero the rest (optional but often useful)
  for (int i = sz; i < NUM_MOTORS; i++) {
    drive_buf[i] = 0;
  }
}

// Enhanced timer callback with interrupt protection: apply PWMs and publish encoders + echo
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;

  // Prepare and publish encoder message
  enc_msg.data.data = enc_feed;
  enc_msg.data.size = NUM_ENC;
  enc_msg.data.capacity = NUM_ENC;

  rcl_publish(&publisher, &enc_msg, NULL);

  // Prepare and publish echo message using same backing buffer (drive_buf)
  pwm_echo.data.data = drive_buf;        // <<< KEY: use the same populated array
  pwm_echo.data.size = NUM_MOTORS;      // <<< KEY: set size
  pwm_echo.data.capacity = NUM_MOTORS;

  rcl_publish(&echo_publisher, &pwm_echo, NULL);
}

void setup() {
  // Setup motor pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PWMpin[i], OUTPUT);
    pinMode(dirpin[i], OUTPUT);
    analogWrite(PWMpin[i], 0);  // Initialize to zero
    digitalWrite(dirpin[i], defaultdir[i]);
  }

  // Setup encoder pins and interrupts
  for (int i = 0; i < NUM_ENC; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(encA[0]), callback_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), callback_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[2]), callback_2, CHANGE);

  delay(100);
  Serial.println("micro-ROS bevel arm controller initializing...");

  set_microros_transports();
  Serial.begin(115200); // keep your transport call (ensure agent is running)
  delay(2000);  // Wait for micro-ROS agent (tweak if necessary)

  // Initialize messages
  std_msgs__msg__Int32MultiArray__init(&pwm_int);
  std_msgs__msg__Float32MultiArray__init(&enc_msg);
  std_msgs__msg__Int32MultiArray__init(&pwm_echo);

  // Set ROS Domain ID via args
  const char * args[] = {"--ros-domain-id", "5"};
  int argc = 2;

  // Initialize ROS components
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, argc, args, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microros_bevel_arm", "", &support));

  rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();

  rmw_qos_profile_t my_qos = rmw_qos_profile_default;
  my_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  my_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  my_qos.depth = 10;
  sub_ops.qos = my_qos;
 
  // Subscriber
  RCCHECK(rcl_subscription_init(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "stm_write",
    &sub_ops));

  // Encoder publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "enc_arm"));

  // Echo publisher
  RCCHECK(rclc_publisher_init_default(
    &echo_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "stm_write_echo"));

  // Attach backing buffer for receive/publish (like your working drive code)
  // pwm_int is the receive buffer for subscription; rclc will fill it.
  pwm_int.data.data = drive_buf;   // <<< use the same static backing array
  pwm_int.data.size = 0;
  pwm_int.data.capacity = NUM_MOTORS;

  // Also set pwm_echo defaults to same array (we reassign before publish too)
  pwm_echo.data.data = drive_buf;
  pwm_echo.data.size = 0;
  pwm_echo.data.capacity = NUM_MOTORS;

  // enc_msg buffer
  enc_msg.data.data = enc_feed;
  enc_msg.data.size = 0;
  enc_msg.data.capacity = NUM_ENC;

  // Setup timer
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(TIMER_PERIOD_MS),
    timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &pwm_int,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("Initialization complete");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  for (int i = 0; i < NUM_MOTORS; i++) {
    int pwm_value = abs(drive_buf[i]);
    pwm_value = constrain(pwm_value, 0, MAX_PWM_VALUE);
    
    if (drive_buf[i] >= 0) {
      analogWrite(PWMpin[i], pwm_value);
      digitalWrite(dirpin[i], defaultdir[i]);
    } else {
      analogWrite(PWMpin[i], pwm_value);
      digitalWrite(dirpin[i], 1 - defaultdir[i]);
    }
  }

  // Protected encoder reading
  long enc_copy[NUM_ENC];
  noInterrupts();
  for (int j = 0; j < NUM_ENC; j++) enc_copy[j] = enc_pos[j];
  bool updated = enc_updated;
  enc_updated = false;
  interrupts();

  for (int j = 0; j < NUM_ENC; j++) {
    enc_feed[j] = enc_copy[j] * ENC_TO_DEGREES;
  }
}