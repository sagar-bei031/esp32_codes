#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define Q_DEBUG 0
#define A_DEBUG 1
#define G_DEBUG 2
#define E_DEBUG 3

#define DEBUG

#define DOMAIN_ID 1
#define INTERRUPT_PIN 18
#define START_BYTE 0xA5
#define LED_PIN 19

sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_init_options_t init_options;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

MPU6050 mpu;
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

void error_loop() {
  int i=0;
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    if(++i = 10)
      break;
  }
  esp_restart();
}

// void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
//     msg.data++;
//   }
// }

uint32_t last_published_tick = 0;

void setup() {
  Serial.begin(9600);

  // Micro_ros setup
  set_microros_wifi_transports("Robotics_2G", "muavang_aug25", "10.100.40.118", 8888);
  //GPIO setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  Serial.println("Wifi Done");

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));

  //create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "imu_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

  Serial.println("\nimu_node is runnning...");

// IMU Setup
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-31);
  mpu.setYAccelOffset(-1401);
  mpu.setZAccelOffset(2153);
  mpu.setXGyroOffset(365);
  mpu.setYGyroOffset(211);
  mpu.setZGyroOffset(-99);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
  }

  Serial.println("DMP Ready.");

  imu_msg.header.stamp.sec = 0;
  imu_msg.header.stamp.nanosec = 0;
  imu_msg.header.frame_id.capacity = 100;
  imu_msg.header.frame_id.data = (char*)malloc(imu_msg.header.frame_id.capacity);
  char header_data[] = "base_link";
  memcpy(imu_msg.header.frame_id.data, header_data, strlen(header_data) + 1);
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorFloat gravity;  // [x, y, z]            gravity vector
    VectorInt16 gyro;
    VectorInt16 a;
    // VectorInt16 aa;
    // VectorInt16 aaa;
    float ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    const float g = 9.80665;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    q.y = -q.y;
    q.z = -q.z;

    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&a, fifoBuffer);
    // mpu.dmpGetLinearAccel(&aa, &a, &gravity);
    // mpu.dmpGetLinearAccelInWorld(&aaa, &aa, &q);

    unsigned long microsec = micros();
    unsigned long sec = microsec / 1000000;
    unsigned long nsec = (microsec - sec * 1000000) * 1000;
    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nsec;

    // Assign header values to the imu_msg
    imu_msg.orientation.w = q.w;
    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;

    imu_msg.orientation_covariance[0] = 0.001218;
    imu_msg.orientation_covariance[1] = 0.0;
    imu_msg.orientation_covariance[2] = 0.0;
    imu_msg.orientation_covariance[3] = 0.0;
    imu_msg.orientation_covariance[4] = 0.001218;
    imu_msg.orientation_covariance[5] = 0.0;
    imu_msg.orientation_covariance[6] = 0.0;
    imu_msg.orientation_covariance[7] = 0.0;
    imu_msg.orientation_covariance[8] = 0.001218;

    /* AFS_SEL    Full Scale Range   LSB Sensitivity
     * 0           +/- 250  deg/s    131  LSB/deg/s
     * 1           +/- 500  deg/s    65.5 LSB/deg/s
     * 3           +/- 1000 deg/s    32.8 LSB/deg/s
     * 0           +/- 2000 deg/s    16.4 LSB/deg/s   #
     */
    imu_msg.angular_velocity.x = (gyro.x * 2000.0 * PI) / (16.4 * 180.0);
    imu_msg.angular_velocity.y = (gyro.y * 2000.0 * PI) / (16.4 * 180.0);
    imu_msg.angular_velocity.z = (gyro.z * 2000.0 * PI) / (16.4 * 180.0);

    /* AFS_SEL    Full Scale Range   LSB Sensitivity
     * 0           +/- 2g            16384 LSB/g      #
     * 1           +/- 4g            8192  LSB/g
     * 2           +/- 8g            4096  LSB/g
     * 3           +/- 16g           2048  LSB/g
     */
    imu_msg.linear_acceleration.x = (a.x * 2.0 * g) / (16384.0);
    imu_msg.linear_acceleration.y = (a.y * 2.0 * g) / (16384.0);
    imu_msg.linear_acceleration.z = (a.z * 2.0 * g) / (16384.0);

    // imu_msg.linear_acceleration.x = (gravity.x * g);
    // imu_msg.linear_acceleration.y = (gravity.y * g);
    // imu_msg.linear_acceleration.z = (gravity.z * g;

    imu_msg.angular_velocity_covariance[0] = 1.314664;
    imu_msg.angular_velocity_covariance[1] = 0.0;
    imu_msg.angular_velocity_covariance[2] = 0.0;
    imu_msg.angular_velocity_covariance[3] = 0.0;
    imu_msg.angular_velocity_covariance[4] = 1.314664;
    imu_msg.angular_velocity_covariance[5] = 0.0;
    imu_msg.angular_velocity_covariance[6] = 0.0;
    imu_msg.angular_velocity_covariance[7] = 0.0;
    imu_msg.angular_velocity_covariance[8] = 1.314664;

    imu_msg.linear_acceleration_covariance[0] = 0.0004;
    imu_msg.linear_acceleration_covariance[1] = 0.0;
    imu_msg.linear_acceleration_covariance[2] = 0.0;
    imu_msg.linear_acceleration_covariance[3] = 0.0;
    imu_msg.linear_acceleration_covariance[4] = 0.0004;
    imu_msg.linear_acceleration_covariance[5] = 0.0;
    imu_msg.linear_acceleration_covariance[6] = 0.0;
    imu_msg.linear_acceleration_covariance[7] = 0.0;
    imu_msg.linear_acceleration_covariance[8] = 0.0004;

    Serial.print("angular xyz: ");
    Serial.print(imu_msg.angular_velocity.x);
    Serial.print(' ');
    Serial.print(imu_msg.angular_velocity.y);
    Serial.print(' ');
    Serial.print(imu_msg.angular_velocity.z);

    // #if defined DEBUG && DEBUG == A_DEBUG
    Serial.print("acc xyz: ");
    Serial.print(imu_msg.linear_acceleration.x);
    Serial.print(' ');
    Serial.print(imu_msg.linear_acceleration.y);
    Serial.print(' ');
    Serial.print(imu_msg.linear_acceleration.z);
    // #elif defined DEBUG && DEBUG == G_DEBUG
        Serial.print("   g xyz: ");
        Serial.print(gravity.x);
        Serial.print(' ');
        Serial.print(gravity.y);
        Serial.print(' ');
        Serial.print(gravity.z);
    // #elif defined DEBUG && DEBUG == Q_DEBUG
        Serial.print("  wxyz: ");
        Serial.print(q.w);
        Serial.print(' ');
        Serial.print(q.x);
        Serial.print(' ');
        Serial.print(q.y);
        Serial.print(' ');
        Serial.print(q.z);
    // #elif defned DEBUG && DEBUG == E_DEBUG
    Serial.print("   ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    // #endif

    Serial.print(mpu.getFullScaleAccelRange());


    // Publish message to topic
    uint32_t dtick = millis() - last_published_tick;
    if (dtick >= 50) {
      RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
      last_published_tick = millis();
    }
  }
}
