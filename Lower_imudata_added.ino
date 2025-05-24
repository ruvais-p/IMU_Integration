#define USE_USBCON

#include <Encoder.h>

#define Encoder_output_A1 53
#define Encoder_output_B1 52
#define Encoder_output_A2 49
#define Encoder_output_B2 47
#define Encoder_output_A3 15 // 44
#define Encoder_output_B3 16 // 42
#define Encoder_output_A4 39
#define Encoder_output_B4 37
#define Encoder_output_A5 35
#define Encoder_output_B5 33
#define Encoder_output_A6 31
#define Encoder_output_B6 29
#define Encoder_output_A7 36
#define Encoder_output_B7 38
#define Encoder_output_A8 32
#define Encoder_output_B8 30
#define Encoder_output_A9 26
#define Encoder_output_B9 28
#define Encoder_output_A10 24
#define Encoder_output_B10 22
#define front_left_home
#define tolerance_tick 1
#define const_speed 255
#define max_turning_speed 25
#define min_turning_speed 5
#define total_angle 360
#define max_lock_torque 1
#define slow_indicator_tick 1000
#define servo_constant 2
#define red A4
#define green A5
#define yellow A6

#define tolerance_tick 1
#define const_speed 255
#define max_turning_speed 25
#define min_turning_speed 5
#define total_angle 360
#define max_lock_torque 1
#define slow_indicator_tick 1000
#define rpm_constant 1848.0 / 18.0
#define rpm 1

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "CytronMotorDriver.h"
#include <std_msgs/String.h>
#include <Wire.h>
#include <sensor_msgs/Imu.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();


//RPM calculation parameters
#define total_tick_1Rotation 3000
#define rpm_delay 10

// Define the speed parameters
#define noise_threshold 10
#define delta_speed 5
#define number_of_encoders 4 // this defines the number of encoders used

String currentSteeringMode = "default"; // Initial mode is "default"


ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel;
std_msgs::String pulseCountsMsg;
std_msgs::String rpm_msg;
sensor_msgs::Imu imu_msg;



CytronMD motor_front_right(PWM_DIR, 2, 8);
CytronMD motor_front_left(PWM_DIR, 4, 9);
CytronMD motor_middle_right(PWM_DIR, 3, 10); // neg
CytronMD motor_middle_left(PWM_DIR, 5, 11);  // neg
CytronMD motor_back_right(PWM_DIR, 6, 12);
CytronMD motor_back_left(PWM_DIR, 7, 13); // neg

//CytronMD motor_front_left_steering(PWM_DIR, 50, A1);
//CytronMD motor_back_left_steering(PWM_DIR, 52, A0); // reverse
//CytronMD motor_back_right_steering(PWM_DIR, 48, A2); // reverse
//CytronMD motor_front_right_steering(PWM_DIR, 46, A3); // forward


const int pulsesPerRevolution = 40000; // Adjust this based on your encoder specs

const float wheelBase = 0.2;    // Replace with your robot's wheelbase (distance between wheels)
const float wheelRadius = 0.05; // Replace with your wheel radius

int reset_counter = 0;


volatile int turnfactor = 0;

int tolerance(int a, int b)
{
  if (abs(a - b) < tolerance_tick)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

float variance(float velocity, float meanVelocity) // used for finding the RPM variance
{
  return abs(meanVelocity - velocity);
}

float rpm_value[number_of_encoders] = {0};

void getrpm(unsigned long Timediff, long int delta_tick[]) //this function is used to find the RPM
{
  for (int i = 0; i < number_of_encoders; i++)
  {
    double num = delta_tick[i] * 60000;
    double deno = total_tick_1Rotation * Timediff;
    rpm_value[i] = num / deno;
  }
}

void steeringModeCallback(const std_msgs::String &msg)
{
  currentSteeringMode = msg.data;
}

void cmdVelCallback(const geometry_msgs::Twist &msg)
{
  float linearVel = msg.linear.x;
  float angularVel = msg.angular.z;

  if (currentSteeringMode == "default")
  {
    int linear_Val = linearVel * 255 / 0.26;
    int angular_Val = angularVel * 255 / 1.82;

    int leftSpeed = (linear_Val - angular_Val);
    int rightSpeed = (linear_Val + angular_Val);

    motion(leftSpeed, rightSpeed);
  }

  if (linearVel == 0 && angularVel == 0)
  {
    digitalWrite(red, HIGH);
    digitalWrite(yellow, LOW);
    digitalWrite(green, HIGH);
  }
  else
  {
    digitalWrite(red, HIGH);
    digitalWrite(yellow, HIGH);
    digitalWrite(green, LOW);
  }
}

void motion(int leftSpeed, int rightSpeed)
{
  Serial.print(leftSpeed);
  motor_front_left.setSpeed(leftSpeed);  //-leftSpeed
  motor_back_left.setSpeed(-leftSpeed);  // leftSpeed
  motor_middle_left.setSpeed(-leftSpeed);
  motor_front_right.setSpeed(rightSpeed);
  motor_back_right.setSpeed(rightSpeed);
  motor_middle_right.setSpeed(-rightSpeed);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);
ros::Publisher rpm_pub("/rpm", &rpm_msg);

ros::Publisher pulseCountsPub("pulse_counts", &pulseCountsMsg);
ros::Publisher imu_pub("imu_data", &imu_msg);



ros::Subscriber<std_msgs::String> steeringModeSub("steering_mode", steeringModeCallback);

#define overide_speed 50

String in = "X";
bool override = false;

// Create encoder instances
Encoder back_left_Encoder(Encoder_output_A6, Encoder_output_B6);
//Encoder middle_left_Encoder(Encoder_output_A3, Encoder_output_B3);
Encoder front_left_Encoder(Encoder_output_A5, Encoder_output_B5);
Encoder front_right_Encoder(Encoder_output_A2, Encoder_output_B2);
//Encoder middle_right_Encoder(Encoder_output_A1, Encoder_output_B1);
Encoder back_right_Encoder(Encoder_output_A4, Encoder_output_B4);

Encoder WheelEncoders[number_of_encoders] = {back_left_Encoder, front_left_Encoder, front_right_Encoder, back_right_Encoder};

long prev_A[number_of_encoders] = {0}; // stores the old values of encoders

void setup()
{
  Serial.begin(57600);
  Serial3.begin(115200);
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);

  digitalWrite(red, LOW);
  digitalWrite(yellow, HIGH);
  digitalWrite(green, HIGH);

 

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(pulseCountsPub);
  nh.advertise(rpm_pub);
    nh.advertise(imu_pub);

}


  long delta_tick[number_of_encoders] = {0};
  long diff_delta_tick[number_of_encoders] = {0}; //used to store the change in ticks for each encoders from the mean tick value
  
void loop()
{

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // Read encoder values

  unsigned long startTime = millis();
  long back_left_Encoder_Value = back_left_Encoder.read(); // index 0 
  //long middle_left_Encoder_Value = middle_left_Encoder.read();// index 1
  long front_left_Encoder_Value = front_left_Encoder.read();// index 2
  long front_right_Encoder_Value = front_right_Encoder.read();// index 3
  //long middle_right_Encoder_Value = middle_right_Encoder.read();// index 4
  long back_right_Encoder_Value = back_right_Encoder.read(); // index 5

  long A[number_of_encoders] = {back_left_Encoder_Value, front_left_Encoder_Value, front_right_Encoder_Value, back_right_Encoder_Value}; //array of encoder values

  int direction[number_of_encoders] = {0};

  long meanVelocityForward = 0;
  long meanVelocityBackward = 0;

  for (int i = 0; i < number_of_encoders; i++)
  {
    if (A[i] == prev_A[i])
    {
      direction[i] = 0;
    }
    else if (A[i] > prev_A[i])
    {
      direction[i] = 1;
    }
    else if (A[i] < prev_A[i])
    {
      direction[i] = -1;
    }
    delta_tick[i] = (abs(A[i]) - abs(prev_A[i]));
  }

  for (int i = 0; i < number_of_encoders; i++)
  {
    if (direction[i] == 1)
    {
      meanVelocityForward = meanVelocityForward + delta_tick[i];
    }
    else if (direction[i] == -1)
    {
      meanVelocityBackward = meanVelocityBackward + delta_tick[i];
    }
  }

  meanVelocityForward = meanVelocityForward / number_of_encoders;
  meanVelocityBackward = meanVelocityBackward / number_of_encoders;

  for (int i = 0; i < number_of_encoders; i++)
  {
    if (direction[i] == 1)
    {
      diff_delta_tick[i] = variance(delta_tick[i], meanVelocityForward);
    }
    else if (direction[i] == -1)
    {
      diff_delta_tick[i] = variance(delta_tick[i], meanVelocityBackward);
    }
  }

  delay(rpm_delay);
  for (int i = 0; i < number_of_encoders; i++)
  {
    prev_A[i] = A[i];
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // Create a comma-separated string of pulse counts
  String pulseCountsStr = String(A[0]) + "," +
                          String(A[1]) + "," +
                          String(A[2]) + "," +
                          String(A[3]);

  pulseCountsMsg.data = pulseCountsStr.c_str();
  pulseCountsPub.publish(&pulseCountsMsg);
  



  //override
  if (Serial3.available())
  {
    in = Serial3.readStringUntil('\n');
  }
  if (override || in != "X")
  {

    Serial.println(in);

    override = true;
    if (in[0] == 'w')
    {
      motion(overide_speed, overide_speed);
    }
    else if (in[0] == 's')
    {
      motion(-overide_speed, -overide_speed);
    }
    else if (in[0] == 'a')
    {
      motion(overide_speed, -overide_speed);
    }
    else if (in[0] == 'd')
    {
      motion(-overide_speed, overide_speed);
    }
    else if (in[0] == 'q')
    {
      motion(0, 0);
    }
    return;
  }



  nh.spinOnce();
      unsigned long Timediff = millis() - startTime;
  getrpm(Timediff, delta_tick);



      String rpm_data = String(delta_tick[0]) + "," +
                    String(delta_tick[1]) + "," +
                    String(delta_tick[2]) + "," +
                    String(delta_tick[3]);

  // Publish RPM values as a single string
  rpm_msg.data = rpm_data.c_str();
  rpm_pub.publish(&rpm_msg);

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  imu_msg.linear_acceleration.x = acc.x();
  imu_msg.linear_acceleration.y = acc.y();
  imu_msg.linear_acceleration.z = acc.z();

  imu_msg.angular_velocity.x = gyro.x();
  imu_msg.angular_velocity.y = gyro.y();
  imu_msg.angular_velocity.z = gyro.z();
  imu_pub.publish(&imu_msg);

}
