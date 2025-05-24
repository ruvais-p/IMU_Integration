#include <ros.h>                          // ROS library for Arduino
#include <std_msgs/String.h>              // Standard ROS message types
#include <Wire.h>                         // Arduino's I2C library
#include <sensor_msgs/Imu.h>              // ROS message type for IMU data
#include <Adafruit_Sensor.h>              // Adafruit unified sensor library
#include <Adafruit_BNO055.h>              // Adafruit BNO055 Absolute Orientation Sensor library
#include <utility/imumaths.h>             // Utility library for IMU math operations

/*
  Connections
  ===========
  Connect SCL to SCL of your Controller (check pin diagram)
  Connect SDA to SDA of your Controller (check pin diagram)
  Connect VDD to 3.3V DC
  Connect GROUND to common ground
*/

// Initialize an Adafruit_BNO055 object with I2C address 0x28 and ID 55
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

ros::NodeHandle nh;                                     // Initialize ROS node handle

sensor_msgs::Imu imu_msg;                              // Define IMU message object

ros::Publisher imu_pub("imu_data", &imu_msg);          // Create a publisher for IMU data

void setup() {
  Serial.begin(115200);                               // Start serial communication

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("Could not find a valid BNO055 sensor, check wiring!");
    while (1);  // Loop indefinitely if the sensor is not found
  }

  nh.initNode();                                    // Initialize ROS node
  nh.advertise(imu_pub);                           // Advertise the IMU data publisher
}

void loop() {
  // Get accelerometer, gyroscope, and Euler angle readings from the BNO055 sensor
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Fill in the IMU message with sensor data
  imu_msg.linear_acceleration.x = acc.x();
  imu_msg.linear_acceleration.y = acc.y();
  imu_msg.linear_acceleration.z = acc.z();

  imu_msg.angular_velocity.x = gyro.x();
  imu_msg.angular_velocity.y = gyro.y();
  imu_msg.angular_velocity.z = gyro.z();

  imu_msg.orientation.x = euler.x();
  imu_msg.orientation.y = euler.y();
  imu_msg.orientation.z = euler.z();
  imu_msg.orientation.w = 0.0;  // Placeholder value for orientation quaternion

  imu_pub.publish(&imu_msg);                      // Publish the IMU message
  nh.spinOnce();                                 // Handle ROS callbacks
}

/*****************************************************************************\
| if utility/imumaths.h shows any during compling error then just remove that |
|                                                                             |
\*****************************************************************************/
