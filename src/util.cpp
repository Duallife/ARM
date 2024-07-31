#include "util.h"

SemaphoreHandle_t xMutex;
SemaphoreHandle_t xMutex2;
// double coordinate[3] = {1, 1, 1};
double theta[MOTOR_NUM] = {0, 0, 0, 0, 0, 0};
const double homePosition[MOTOR_NUM] = {0, PI/2, 0, 0, 0, 0};
#ifndef USEROS
  const float flip[6] = {-1.5,1.5,-1.5,-1.6,-1.5,-0.1};
#else
  const float flip[6] = {-1.5,1.5,-1.5,1.6,-1.5,-0.1};
#endif
SimpleRotary rotary(ENA, ENB, SW);
const int maxSpeed = 1000;
const int maxAccel = 1000;

Servo myservo;
int gripperAngle = 0;

// MODE
mode menu = IDLE;
static bool isMoving = false;

// ROS
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
interface__msg__JointAngle jointAngle;

AccelStepper stepper1(1, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper stepper2(1, MOTOR2_STEP, MOTOR2_DIR);
AccelStepper stepper3(1, MOTOR3_STEP, MOTOR3_DIR);
AccelStepper stepper4(1, MOTOR4_STEP, MOTOR4_DIR);
AccelStepper stepper5(1, MOTOR5_STEP, MOTOR5_DIR);
AccelStepper stepper6(1, MOTOR6_STEP, MOTOR6_DIR);
AccelStepper *J[MOTOR_NUM] = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};

bool Running = false;

double DH_table[MOTOR_NUM][4]{
    {0, 0, 0, 0},
    {0, 0, 0, PI/2},
    {0, 0, 91, 0},
    {0, 113.7, 49, PI/2},
    {0, 0, 0, -PI/2},
    {0, WRIST, 0, PI/2}
};

double target[4][4]{
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},                        
  {0, 0, 0, 1},
};

void tasks_init() {
  xMutex = xSemaphoreCreateMutex();
  xMutex2 = xSemaphoreCreateMutex();
  #ifdef USEROS
    tftPrint("ROS USE", 1);
    xTaskCreatePinnedToCore(microRosTask_handler, "microRosTask", 10240, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(motorTask_handler, "motorTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(motor_handler, "motor", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sideTask_handler, "sideTask", 4096, NULL, 1, NULL, 1); vTaskDelay(100);
  #else
    tftPrint("Serial USE", 1);
    xTaskCreate(serialTask_handler, "serialTask", 4096, NULL, 2, NULL); vTaskDelay(100);
    xTaskCreate(sideTask_handler, "sideTask", 4096, NULL, 1, NULL); vTaskDelay(100);
  #endif
  vTaskDelay(1000);
}

void util_init() {
    for (int i = 0; i < MOTOR_NUM; i++){
      if (i == 1){
        #ifndef USEROS
        J[i]->setCurrentPosition(6000);
        #else
        J[i]->setCurrentPosition(0);
        #endif
      }
      else{
        J[i]->setCurrentPosition(0); 
      }
      J[i]->setMaxSpeed(maxSpeed);
      J[i]->setAcceleration(maxAccel);
    }
    pinMode(ENABLEPIN, OUTPUT);
    digitalWrite(ENABLEPIN, LOW);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50);    // standard 50 hz servo
    myservo.attach(SERVO, 1000, 2000); // attaches the servo on pin 18 to the servo object
    tftPrint("Checking Motors....", 1);
    Serial.println("Checking Motors....");
    motorCheck();
    tftPrint("Util Init Done", 1);
    Serial.println("Util Init Done");

}

void ros_init() {
  menu = ROS;
  set_microros_serial_transports(Serial);
  delay(2000);
  allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) { tftPrint("Failed to initialize ROS 2 support structure.",1);}
  rclc_node_init_default(&node, "microros_joint_subscriber", "", &support);
  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interface, msg, JointAngle),
      "command");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &jointAngle, &callback_subscription, ON_NEW_DATA);
  tftPrint("ROS Init Done", 1);
}

// ///FUNCTIONS///////////////////////////////////////////////////////

void callback_subscription(const void *msgin)
{
  static const float TOLERANCE = 0.01;
  const interface__msg__JointAngle *msg = (const interface__msg__JointAngle *)msgin;
  float joint_angle[6] = {msg->joint1, msg->joint2, msg->joint3, msg->joint4, msg->joint5, msg->joint6}; 
  for (int i = 0; i < MOTOR_NUM; i++){
    if (fabs(joint_angle[i] - theta[i]) > TOLERANCE){
      theta[i] = joint_angle[i];
    }
  }
  gripperAngle = msg->gripper;
  cleanScreen();
}

void servoControl(int angle){
  myservo.write(angle);
}

void motorCheck(){
  int dist = 300;
  int dt[6] = {int(dist*flip[0]), int(dist*flip[1]), int(dist*flip[2]), int(dist*flip[3]), int(dist*flip[4]), int(dist*flip[5])};
  for (int i = 0; i < MOTOR_NUM; i++){
    J[i]->move(dt[i]);
  }
  while (abs(J[0]->distanceToGo()) || abs(J[1]->distanceToGo()) || abs(J[2]->distanceToGo()) || 
  abs(J[3]->distanceToGo()) || abs(J[4]->distanceToGo()) || abs(J[5]->distanceToGo())){
    {
      for (int i = 0; i < MOTOR_NUM; i++){
        J[i]->run();
      }
    } 
  }

  for (int i = 0; i < MOTOR_NUM; i++){
    J[i]->move(-dt[i]);
  }
  while (abs(J[0]->distanceToGo())|| abs(J[1]->distanceToGo()) || abs(J[2]->distanceToGo()) || 
  abs(J[3]->distanceToGo()) || abs(J[4]->distanceToGo()) || abs(J[5]->distanceToGo())){
    {
      for (int i = 0; i < MOTOR_NUM; i++){
        J[i]->run();
      }
    } 
  }
}

void printTarget(){
  Serial.println("Target Theta: ");
  printMatrix(theta, 1, 6);
  
  Serial.println("Target Pose: ");
  printMatrix(*target, 4, 4);
  tftPrint("Target Theta: ", 1);
  for (int i = 0; i < MOTOR_NUM; i++){
    tftPrint("Motor " + String(i) + " : " + String(theta[i]), 1);
  }
}

void Homing(){
  for (int i = 0; i < MOTOR_NUM; i++){
    theta[i] = homePosition[i];
  }
  runMotors();
}

void runMotors(){
  xTaskCreatePinnedToCore(motorRunTask_handler, "motorRunTask", 4096, NULL, 1, NULL, 1);
  vTaskDelay(100);
}

void setTarget(double input[4][4], double target[4][4]){
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            target[i][j] = input[i][j];
        }
    }
}

void setTheta(double t[MOTOR_NUM]){
    for (int i = 0; i < MOTOR_NUM; i++){
        DH_table[i][0] = t[i];
        theta[i] = t[i];
    }
}

void forceStop(){
    for (int i = 0; i < MOTOR_NUM; i++){
        J[i]->stop();
    }
    digitalWrite(ENABLEPIN, HIGH);
}

void getCurrentPosition(){
  Serial.println("Current Position: ");
  double currentPos[MOTOR_NUM] = {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < MOTOR_NUM; i++){
    currentPos[i] = J[i]->currentPosition() * PI / 8000 / flip[i];
  }
  printMatrix(currentPos, 1, 6);
  cleanScreen();
  for (int i = 0; i < MOTOR_NUM; i++){
    tftPrint("Motor " + String(i) + " : " + String(currentPos[i]), 1);
  }
}

///TASK////////////////////////////////////////////////////////

void sideTask(){
    byte enc = rotary.rotate();
    byte button = rotary.pushType(500);
    if (enc == 1){
        encoder(true);
    }
    if (enc == 2){
        encoder(false);
    }
    if (button == 1){
        click();
    }
    if (button == 2){
        longPress();
    }
}

void microRosTask(){
  vTaskDelay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void motorSet(){
  float dist[6] = {0, 0, 0, 0, 0, 0};
  float maxDist = 0;
  for (int i = 0; i < MOTOR_NUM; i++)
  { 
    J[i]->moveTo(theta[i] / PI * 8000 * flip[i]);
  }
  for (int i = 0; i < MOTOR_NUM; i++)
  { 
    dist[i] = abs(J[i]->distanceToGo()); 
    if (dist[i] > maxDist){ maxDist = dist[i];}
  }
  for (int i = 0; i < MOTOR_NUM; i++)
  { 
    if (dist[i]/maxDist > 0.05){
      J[i]->setMaxSpeed(dist[i]/maxDist * maxSpeed);
      J[i]->setAcceleration(dist[i]/maxDist * maxAccel);
    }
    else {
      J[i]->setMaxSpeed(maxSpeed * 0.05);
      J[i]->setAcceleration(maxAccel * 0.05);
    }
  }
  myservo.write(gripperAngle);
  tftPrint("Gripper: " + String(gripperAngle), 1);
  for (int i = 0; i < MOTOR_NUM; i++){
    tftPrint("Motor " + String(i) + " : " + String(theta[i]) + " Sp " + String(dist[i]/maxDist * maxAccel), 1);
  }
}

void motorTask(){
  while (abs(J[0]->distanceToGo())|| abs(J[1]->distanceToGo()) || abs(J[2]->distanceToGo()) || 
  abs(J[3]->distanceToGo()) || abs(J[4]->distanceToGo()) || abs(J[5]->distanceToGo())){
    for (int i = 0; i < MOTOR_NUM; i++)
    {
      J[i]->run();
    } 
  }
}

void motorRunTask(){
  float dist[6] = {0, 0, 0, 0, 0, 0};
  float maxDist = 0;
  for (int i = 0; i < MOTOR_NUM; i++)
  { 
    int tmp = 0;
    tmp = (theta[i] / PI) * 8000 * flip[i];
    dist[i] = abs(tmp - J[i]->currentPosition());
    if (dist[i] > maxDist){ maxDist = dist[i]; }
    J[i]->moveTo(tmp);
  }
  Serial.println("Max Distance: " + String(maxDist));
  for (int i = 0; i < MOTOR_NUM; i++)
  { 
    J[i]->setMaxSpeed(dist[i]/maxDist * maxSpeed);
    J[i]->setAcceleration(dist[i]/maxDist * maxAccel);
  }
  while (abs(J[0]->distanceToGo())|| abs(J[1]->distanceToGo()) || abs(J[2]->distanceToGo()) || 
  abs(J[3]->distanceToGo()) || abs(J[4]->distanceToGo()) || abs(J[5]->distanceToGo())){
    {
      for (int i = 0; i < MOTOR_NUM; i++){
        J[i]->run();
      }
    } 
  }
  Running = false;
}

///HANDLER///////////////////////////////////////////////////////

void sideTask_handler(void *pvParameters) {
    while (1) {
    if (xSemaphoreTake(xMutex2, portMAX_DELAY) == pdTRUE) {
        sideTask();
        xSemaphoreGive(xMutex2);
    }
    vTaskDelay(5); // Adjust delay as needed
  }
}

void serialTask_handler(void *pvParameters) {
    while (1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        serialTask();
        xSemaphoreGive(xMutex);
    }
    vTaskDelay(5); // Adjust delay as needed
  }
}

void motorRunTask_handler(void * pvPrarameters){
    // Serial.println("Motor Run Task Started");
    Running = true;
    while (Running) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        motorRunTask();
        xSemaphoreGive(xMutex);
    }
    Serial.println("Motor done");
    tftPrint("MOTOR DONE", true);
    vTaskDelete(NULL);
  }
}

void microRosTask_handler(void *pvParameters) {
    while (1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        microRosTask();
        xSemaphoreGive(xMutex);
    }
    vTaskDelay(5);
  }
}


void motor_handler(void *pvParameters) {
    while (1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        motorSet();
        xSemaphoreGive(xMutex);
    }
    vTaskDelay(50); // Adjust delay as needed
  }
}

void motorTask_handler(void *pvParameters) {
    while (1) {
    if (xSemaphoreTake(xMutex2, portMAX_DELAY) == pdTRUE) {
        motorTask();
        xSemaphoreGive(xMutex2);
    }
    vTaskDelay(5); // Adjust delay as needed
  }
}
