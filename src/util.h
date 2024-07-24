#ifndef UTIL_H
#define UTIL_H

// #define EXTRAI2C
#define USEROS

#include <Arduino.h>
#include <AccelStepper.h>
#include <SimpleRotary.h>
#include <TFT_eSPI.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <sensor_msgs/msg/joint_state.h>
// #include <vector>
#include <interface/msg/joint_angle.h>


#include "kinematic.h"
#include "menuControl.h"
#include "serial.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// parameter
#define MOTOR_NUM 6
// communication
#define maxLength 50
#define maxElements 20

#define TX 1
#define RX 3
// pins
#define ENA 39
#define ENB 36
#define SW 34
#define MOTOR1_STEP 21
#define MOTOR1_DIR 19
#define MOTOR2_STEP 23
#define MOTOR2_DIR 22
#define MOTOR3_STEP 17
#define MOTOR3_DIR 16
#define MOTOR4_STEP 18
#define MOTOR4_DIR 5
#define MOTOR5_STEP 2
#define MOTOR5_DIR 15
#define MOTOR6_STEP 4
#define MOTOR6_DIR 0

#define servo 33

#define ENABLEPIN 32


enum mode {IDLE, ForK, InvK, ROS};
extern mode menu;

extern double DH_table[6][4];

extern AccelStepper *J[MOTOR_NUM];
extern double theta[MOTOR_NUM];
extern double target[4][4];
extern const int maxAccel;
extern const int maxSpeed;

void tasks_init();
void util_init();
void ros_init();

void motorCheck();
void Homing();
void printTarget();
void runMotors();
void forceStop();
void setTheta(double t[MOTOR_NUM]);
void setTarget(double input[4][4], double target[4][4]);
void callback_subscription(const void* msgin);
void getCurrentPosition();

void motorTask_handler(void *pvParameters);
void sideTask_handler(void *pvParameters);
void serialTask_handler(void *pvParameters);
void microRosTask_handler(void *pvParameters);
void motorRunTask_handler(void *pvParameters);
void motor_handler(void *pvParameters);

#endif // UTIL_H