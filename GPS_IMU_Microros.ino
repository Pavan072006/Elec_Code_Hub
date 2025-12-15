#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>

// ------------------- micro-ROS includes -------------------
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/imu.h>

// ------------------- GPS setup -------------------
#define SDA_GPS 17
#define SCL_GPS 16

SFE_UBLOX_GNSS myGNSS;
TwoWire I2CBusGPS = TwoWire(1);

// ------------------- IMU setup -------------------
#define SDA_IMU 35
#define SCL_IMU 36

Adafruit_MPU6050 mpu;
TwoWire I2CBusIMU = TwoWire(0);

// ------------------- micro-ROS setup -------------------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t gps_pub;
rcl_publisher_t imu_pub;

sensor_msgs__msg__NavSatFix gps_msg;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rcl_timer_t timer;

// ------------------- Timer callback -------------------
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  // ==== Read GPS ====
  double lat = myGNSS.getLatitude() / 1e7;
  double lon = myGNSS.getLongitude() / 1e7;
  double alt = myGNSS.getAltitude() / 1000.0;

  gps_msg.latitude = lat;
  gps_msg.longitude = lon;
  gps_msg.altitude = alt;
  gps_msg.position_covariance[0] = 0.0;
  gps_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
  gps_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
  gps_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

  rcl_publish(&gps_pub, &gps_msg, NULL);

  // ==== Read IMU ====
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Fill IMU message
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;

  imu_msg.orientation.x = 0.0;  // Not computed
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  rcl_publish(&imu_pub, &imu_msg, NULL);
}

// ------------------- Arduino setup -------------------
void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("Initializing GPS + IMU + micro-ROS...");

  // ------------------- micro-ROS transport -------------------
  set_microros_transports();
  delay(2000);

  // ------------------- GPS init -------------------
  pinMode(SDA_GPS, INPUT_PULLUP);
  I2CBusGPS.begin(SDA_GPS, SCL_GPS, 100000);


  Serial.println("Initializing GNSS...");
  if (!myGNSS.begin(I2CBusGPS)) {
    while (1) {
      Serial.println("❌ GNSS not found!");
      delay(1000);
    }
  }
  Serial.println("✅ GNSS connected.");

  // ------------------- IMU init -------------------
  I2CBusIMU.begin(SDA_IMU, SCL_IMU);
  pinMode(SDA_IMU, INPUT_PULLUP);
  pinMode(SCL_IMU, INPUT_PULLUP);

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &I2CBusIMU)) {
    while (1) {
      Serial.println("❌ Failed to find MPU6050 chip!");
      delay(1000);
    }
  }
  Serial.println("✅ MPU6050 connected.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // ------------------- micro-ROS init -------------------
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_s3_node", "", &support);

  // GPS Publisher
  rclc_publisher_init_default(
      &gps_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
      "gps/fix");

  // IMU Publisher
  rclc_publisher_init_default(
      &imu_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data_raw");

  // Timer (every 500ms)
  const unsigned int timer_timeout = 500;
  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  Serial.println("✅ Setup complete. Publishing data to ROS 2 topics...\n");
}

// ------------------- Loop -------------------
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
