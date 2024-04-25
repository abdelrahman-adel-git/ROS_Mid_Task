#include <Arduino.h>

#include <L298N.h>
#include <MPU6050_Light.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <math.h>

// Pin definition
#define INL1 22
#define INL2 24
#define ENL 8

#define INR1 28
#define INR2 26
#define ENR 7

const int PIN_ENCOD_A_MOTOR_LEFT = 21; // A channel for encoder of left motor
const int PIN_ENCOD_B_MOTOR_LEFT = 52; // B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3; // A channel for encoder of right motor
const int PIN_ENCOD_B_MOTOR_RIGHT = 4; // B channel for encoder of right motor

volatile float pos_left = 0;  // Left motor encoder position
volatile float pos_right = 0; // Right motor encoder position

// int left_V_ros;
// int right_V_ros;

float left_V_ros_recieved;
float right_V_ros_recieved;

float PWM_left;
float PWM_right;

const int cpr = 520;

float dt = 0.1;

float V_Max = 13;

float current_micros;
float prev_micros;


volatile long prevEncoderCountLeft = 0; // previous count for calculating speed
volatile long prevEncoderCountRight = 0; // previous count for calculating speed

float angularSpeedLeft;
float angularSpeedRight;
// Create one motor instance
L298N motor_right(ENR, INR1, INR2);
L298N motor_left(ENL, INL1, INL2);

void encoderLeftMotor()
{
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT))
    pos_left++;
  else
    pos_left--;
}

// Right motor encoder counter
void encoderRightMotor()
{
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT))
    pos_right--;
  else
    pos_right++;
}

ros::NodeHandle nh;

void callBackFunctionMotorLeft(const std_msgs::Int32 &left_V_ros)
{
  left_V_ros_recieved = left_V_ros.data;
}

void callBackFunctionMotorRight(const std_msgs::Int32 &right_V_ros)
{
  right_V_ros_recieved = right_V_ros.data;
}

ros::Subscriber<std_msgs::Int32> leftMotorROSSSUB("vl", &callBackFunctionMotorLeft);

ros::Subscriber<std_msgs::Int32> rightMotorROSSSUB("vr", &callBackFunctionMotorRight);

std_msgs::Int32 right_encoder;
ros::Publisher right_encoder_Publisher("encoder_right", &right_encoder);

std_msgs::Int32 left_encoder;
ros::Publisher left_encoder_Publisher("encoder_left", &left_encoder);

std_msgs::Int32 MPU_reading;
ros::Publisher MPU_Publisher("MPU", &MPU_reading);

MPU6050 mpu(Wire);

void setup()
{
  pinMode(INR1, OUTPUT);
  pinMode(INR2, OUTPUT);
  pinMode(ENR, OUTPUT);
  pinMode(INL1, OUTPUT);
  pinMode(INL2, OUTPUT);
  pinMode(ENL, OUTPUT);

  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH); // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(19), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH); // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), encoderRightMotor, RISING);

  byte status = mpu.begin();
  mpu.calcOffsets(true, true); // gyro and accelero

  nh.getHardware()->setBaud(9600);
  nh.initNode();


  nh.advertise(right_encoder_Publisher);
  nh.advertise(left_encoder_Publisher);
  nh.advertise(MPU_Publisher);

  nh.subscribe(leftMotorROSSSUB);
  nh.subscribe(rightMotorROSSSUB);

  prev_micros = 0;
}

void loop()
{
  nh.spinOnce();

  mpu.update();

  // Set the speed based on the received velocity
  int right_speed = (right_V_ros_recieved / V_Max) * 255;
  int left_speed = (left_V_ros_recieved / V_Max) * 255;

  motor_right.setSpeed(abs(right_speed) * 0.7);
  motor_left.setSpeed(abs(left_speed));

  /////////////////////////////////////////////////////// Motors /////////////////////////////////////////////
  // Set the direction based on the sign of the received velocity
  if (right_speed > 0)
  {
    if (right_speed < 4 && right_speed > 1)
    {
      motor_right.setSpeed(4);
    }
    // If the received velocity is positive, set the motor to move forward
    motor_right.forward();
  }
  else if (right_speed < 0)
  {
    if (right_speed > -4 && right_speed < -1)
    {
      motor_right.setSpeed(4);
    }
    // If the received velocity is negative, set the motor to move backward
    motor_right.backward();
  }
  else if (abs(right_speed) <= 1)
  {
    // If the received velocity is very small, stop the motor
    motor_right.setSpeed(0);
    motor_right.stop();
  }

  if (left_speed > 0)
  {
    if (left_speed < 4 && left_speed > 1)
    {
      motor_left.setSpeed(4);
    }
    // If the received velocity is positive, set the motor to move forward
    motor_left.forward();
  }
  else if (left_speed < 0)
  {
    if (left_speed > -4 && left_speed < -1)
    {
      motor_left.setSpeed(4);
    }
    // If the received velocity is negative, set the motor to move backward
    motor_left.backward();
  }
  else if (abs(left_speed) <= 1)
  {
    // If the received velocity is very small, stop the motor
    motor_left.setSpeed(0);
    motor_left.stop();
  }
  //////////////////////////////////////////////////////////////////////////////////////////


  current_micros = micros();



    angularSpeedLeft = (pos_left - prevEncoderCountLeft) * (2 * PI / cpr) / (0.100);
    //Serial.print("Angular speed left (rad/s): ");
    //Serial.println(angularSpeedLeft);

    angularSpeedRight = ((pos_right - prevEncoderCountRight) * (2 * PI / cpr) / (0.100));
    Serial.print("Angular speed right (rad/s): ");
    Serial.println(angularSpeedRight);

    right_encoder.data = angularSpeedRight;
    left_encoder.data = angularSpeedLeft;

    right_encoder_Publisher.publish(&right_encoder);
    left_encoder_Publisher.publish(&left_encoder);


  // Get IMU rotational velocity around Z axis in deg/s
  MPU_reading.data = mpu.getGyroZ();

  MPU_Publisher.publish(&MPU_reading);

  // Serial.print("Deg/s");
  // Serial.println(MPU_reading.data);
  //  Serial.println(pos_left/1.444);

  prevEncoderCountLeft = pos_left;
  prevEncoderCountRight = pos_right;

  delay(100);
}