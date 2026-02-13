#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ESP32Encoder.h>
#include <BluetoothSerial.h>
#include <cmath>

// ===============================
// ✅ PHYSICAL + CONTROL PARAMETERS
// ===============================
static const float WHEEL_RADIUS_M = 0.0325f;   // [m]
static const float WHEEL_BASE_M   = 0.33f;     // [m]

// Encoders (half-quad mode! this number MUST match your mode)
static const float MOTOR1_TICKS_PER_ROTATION = 7050.0f;
static const float MOTOR2_TICKS_PER_ROTATION = 7050.0f;

// Control loop timing
static const uint32_t SAMPLE_PERIOD_US = 100000;   // [us] PID update period (100ms)

// Minimum wheel reference (RPS)
static const float MIN_REF_RPS = 0.7f;

// Pi constant
static const float PI_F = 3.14159265358979323846f;

// =========================
// BTS7960 PINS
// =========================
#define MOTOR1_R_EN   13
#define MOTOR1_L_EN   14
#define MOTOR1_RPWM   12
#define MOTOR1_LPWM   17

#define MOTOR2_R_EN   26
#define MOTOR2_L_EN   27
#define MOTOR2_RPWM   25
#define MOTOR2_LPWM   33

// =========================
// Encoder pins (your current wiring)
// =========================
#define MOTOR1_ENCODER_PIN_A 4
#define MOTOR1_ENCODER_PIN_B 5
#define MOTOR2_ENCODER_PIN_A 15
#define MOTOR2_ENCODER_PIN_B 32

BluetoothSerial SerialBT;

// ===============================
// micro-ROS objects
// ===============================
rcl_node_t node = rcl_get_zero_initialized_node();
rclc_support_t support;
rcl_allocator_t allocator;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_publisher_t odom_pub = rcl_get_zero_initialized_publisher();
rcl_publisher_t tf_pub   = rcl_get_zero_initialized_publisher();

rclc_executor_t executor;

nav_msgs__msg__Odometry odom;
tf2_msgs__msg__TFMessage tf_msg;

// ===============================
// PID gains
// ===============================
float Kp = 120;
float Ki = 5;
float Kd = 1.2;

// ===============================
// Control state
// ===============================
float Ts = (float)SAMPLE_PERIOD_US;
float reference1 = 0, reference2 = 0;
float error1 = 0, error1Last = 0;
float error2 = 0, error2Last = 0;
float controllaw1 = 0, controllaw2 = 0;
float integrat1 = 0, integrat2 = 0;
float derevative1 = 0, derevative2 = 0;

// Encoders
volatile long encoderval1 = 0;
volatile long encoderval2 = 0;
float mspeed1 = 0;   // [rev/s]
float mspeed2 = 0;   // [rev/s]
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// Timing
unsigned long prev_pid_time_us = 0;

// Odom integration state
double x = 0, y = 0, theta = 0;

// ✅ Synced ROS epoch timing
int64_t last_ns = 0;

// Debug timing
unsigned int timerc = 0;
unsigned int timerp = 0;

// ===============================
// Helpers / macros
// ===============================
#define RCCHECK(fn) \
  { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }

void error_loop() {
  while (1) { delay(100); }
}

// BTS7960 drive helper (signed u)
static inline void setMotorBTS7960(int r_en, int l_en, int rpwm, int lpwm, float u) {
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);

  int pwm = (int)constrain((int)fabs(u), 0, 255);

  if (pwm == 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
    return;
  }

  if (u > 0) {
    analogWrite(rpwm, pwm);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, pwm);
  }
}

// cmd_vel callback -> references in RPS
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

  const float v = m->linear.x;
  const float w = m->angular.z;

  reference1 = (v + (WHEEL_BASE_M * w) / 2.0f) / (2.0f * PI_F * WHEEL_RADIUS_M);
  reference2 = (v - (WHEEL_BASE_M * w) / 2.0f) / (2.0f * PI_F * WHEEL_RADIUS_M);
}

void setup() {
  set_microros_transports();

  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");
  delay(2000);

  // Motor pins
  pinMode(MOTOR1_R_EN, OUTPUT);
  pinMode(MOTOR1_L_EN, OUTPUT);
  pinMode(MOTOR1_RPWM, OUTPUT);
  pinMode(MOTOR1_LPWM, OUTPUT);

  pinMode(MOTOR2_R_EN, OUTPUT);
  pinMode(MOTOR2_L_EN, OUTPUT);
  pinMode(MOTOR2_RPWM, OUTPUT);
  pinMode(MOTOR2_LPWM, OUTPUT);

  digitalWrite(MOTOR1_R_EN, HIGH);
  digitalWrite(MOTOR1_L_EN, HIGH);
  digitalWrite(MOTOR2_R_EN, HIGH);
  digitalWrite(MOTOR2_L_EN, HIGH);

  analogWrite(MOTOR1_RPWM, 0);
  analogWrite(MOTOR1_LPWM, 0);
  analogWrite(MOTOR2_RPWM, 0);
  analogWrite(MOTOR2_LPWM, 0);

  // Encoders
  encoder1.attachHalfQuad(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B);
  encoder2.attachHalfQuad(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B);
  encoder1.clearCount();
  encoder2.clearCount();

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // subscriber cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // publishers
  RCCHECK(rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"));

  RCCHECK(rclc_publisher_init_default(
    &tf_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));

  // executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // =========================
  // ✅ THE IMPORTANT FIX (from code #2)
  // =========================
  rmw_uros_sync_session(1000);       // sync with agent time
  last_ns = rmw_uros_epoch_nanos();  // init epoch time

  // Odom msg frame names
  odom.header.frame_id.data = (char *)"odom";
  odom.header.frame_id.size = strlen(odom.header.frame_id.data);
  odom.header.frame_id.capacity = odom.header.frame_id.size + 1;

  odom.child_frame_id.data = (char *)"base_link";
  odom.child_frame_id.size = strlen(odom.child_frame_id.data);
  odom.child_frame_id.capacity = odom.child_frame_id.size + 1;

  // TF msg allocate
  tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped *)malloc(sizeof(geometry_msgs__msg__TransformStamped));
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;

  tf_msg.transforms.data[0].header.frame_id.data = (char *)"odom";
  tf_msg.transforms.data[0].header.frame_id.size = strlen(tf_msg.transforms.data[0].header.frame_id.data);
  tf_msg.transforms.data[0].header.frame_id.capacity = tf_msg.transforms.data[0].header.frame_id.size + 1;

  tf_msg.transforms.data[0].child_frame_id.data = (char *)"base_link";
  tf_msg.transforms.data[0].child_frame_id.size = strlen(tf_msg.transforms.data[0].child_frame_id.data);
  tf_msg.transforms.data[0].child_frame_id.capacity = tf_msg.transforms.data[0].child_frame_id.size + 1;

  prev_pid_time_us = micros();
}

void loop() {
  // Receive cmd_vel
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));

  // Bluetooth tuning (optional)
  if (SerialBT.available()) {
    String message = SerialBT.readStringUntil('\n');
    String text = message.substring(0, message.indexOf(' '));
    float number = message.substring(message.indexOf(' ') + 1).toFloat();

    if (text == "speed") {
      reference1 = number;
      reference2 = number;
    } else if (text == "kp") {
      Kp = number;
    } else if (text == "kd") {
      Kd = number;
    } else if (text == "ki") {
      Ki = number;
    } else if (text == "rotateL") {
      reference1 = 0;
      reference2 = number;
    } else if (text == "rotateR") {
      reference1 = number;
      reference2 = 0;
    } else if (text == "reset") {
      ESP.restart();
    }
  }

  // ✅ Get synced ROS epoch time (ns) and compute dt
  int64_t now_ns = rmw_uros_epoch_nanos();
  double dt = (now_ns - last_ns) / 1e9;
  if (dt <= 0.0 || dt > 0.2) dt = 0.02; // safety clamp
  last_ns = now_ns;

  // Encoder read (ticks since last loop)
  long t1 = encoder1.getCount();
  long t2 = encoder2.getCount();
  encoder1.clearCount();
  encoder2.clearCount();

  t1 = -t1;

  // ticks -> wheel speed (rev/s)
  mspeed1 = ((double)t1 / (double)MOTOR1_TICKS_PER_ROTATION) / dt;
  mspeed2 = ((double)t2 / (double)MOTOR2_TICKS_PER_ROTATION) / dt;

  // PID update at Ts
  unsigned long now_us = micros();
  float TS_us = (float)(now_us - prev_pid_time_us);

  if (TS_us >= Ts) {
    prev_pid_time_us = now_us;

    // Reference conditioning
    if (reference1 == 0) integrat1 = 0;
    else if (fabs(reference1) < MIN_REF_RPS) reference1 = MIN_REF_RPS * (fabs(reference1) / reference1);

    if (reference2 == 0) integrat2 = 0;
    else if (fabs(reference2) < MIN_REF_RPS) reference2 = MIN_REF_RPS * (fabs(reference2) / reference2);

    if (reference1 < 0 && controllaw1 > 0) integrat1 = 0;
    else if (reference1 > 0 && controllaw1 < 0) integrat1 = 0;

    if (reference2 < 0 && controllaw2 > 0) integrat2 = 0;
    else if (reference2 > 0 && controllaw2 < 0) integrat2 = 0;

    // Motor 1 PID
    error1 = reference1 - mspeed1;
    derevative1 = error1 - error1Last;
    integrat1 = constrain(integrat1 + error1, -255, 255);
    controllaw1 = error1 * Kp + Ki * integrat1 + Kd * derevative1;
    error1Last = error1;

    // Motor 2 PID
    error2 = reference2 - mspeed2;
    derevative2 = error2 - error2Last;
    integrat2 = constrain(integrat2 + error2, -255, 255);
    controllaw2 = error2 * Kp + Ki * integrat2 + Kd * derevative2;
    error2Last = error2;

    // Drive motors
    float u1 = (reference1 == 0) ? 0.0f : controllaw1;
    float u2 = (reference2 == 0) ? 0.0f : controllaw2;
    setMotorBTS7960(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM, u1);
    setMotorBTS7960(MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM, u2);
  }

  // Wheel speeds (m/s)
  double v1 = (double)mspeed1 * (2.0 * (double)PI_F * (double)WHEEL_RADIUS_M);
  double v2 = (double)mspeed2 * (2.0 * (double)PI_F * (double)WHEEL_RADIUS_M);

  double linear_velocity  = (v1 + v2) / 2.0;
  double angular_velocity = (v1 - v2) / (double)WHEEL_BASE_M;

  // Integrate odom
  x     += linear_velocity * cos(theta) * dt;
  y     += linear_velocity * sin(theta) * dt;
  theta += angular_velocity * dt;

  // Normalize theta (optional but helps)
  if (theta > PI_F) theta -= 2.0 * PI_F;
  if (theta < -PI_F) theta += 2.0 * PI_F;

  // ✅ Stamp with synced epoch time (THIS is the mapping fix)
  odom.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
  odom.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  tf_msg.transforms.data[0].header.stamp.sec     = odom.header.stamp.sec;
  tf_msg.transforms.data[0].header.stamp.nanosec = odom.header.stamp.nanosec;

  // Fill odom
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(theta / 2.0);
  odom.pose.pose.orientation.w = cos(theta / 2.0);

  odom.twist.twist.linear.x  = linear_velocity;
  odom.twist.twist.linear.y  = 0.0;
  odom.twist.twist.angular.z = angular_velocity;

  // Fill TF
  tf_msg.transforms.data[0].transform.translation.x = x;
  tf_msg.transforms.data[0].transform.translation.y = y;
  tf_msg.transforms.data[0].transform.translation.z = 0.0;

  tf_msg.transforms.data[0].transform.rotation.x = 0.0;
  tf_msg.transforms.data[0].transform.rotation.y = 0.0;
  tf_msg.transforms.data[0].transform.rotation.z = sin(theta / 2.0);
  tf_msg.transforms.data[0].transform.rotation.w = cos(theta / 2.0);

  // Publish
  RCCHECK(rcl_publish(&odom_pub, &odom, NULL));
  RCCHECK(rcl_publish(&tf_pub, &tf_msg, NULL));

  // Debug print rate
  timerc = micros();
  if (timerc - timerp > 500000) {
    timerp = timerc;
    SerialBT.print("M1 RPS: "); SerialBT.print(mspeed1);
    SerialBT.print(" | M2 RPS: "); SerialBT.println(mspeed2);
    SerialBT.print("Ref1: "); SerialBT.println(reference1);
    SerialBT.print("v: "); SerialBT.println(linear_velocity);
    SerialBT.print("w: "); SerialBT.println(angular_velocity);
    SerialBT.print("theta: "); SerialBT.println(theta);
    SerialBT.println("----");
  }

  // Run at ~50Hz publish (helps SLAM)
  delay(20);
}