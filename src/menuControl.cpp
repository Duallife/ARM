#include "menuControl.h"

#define bufferSize 9
TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
TFT_eSprite spr = TFT_eSprite(&tft);
int buffer = 0;

void cleanScreen()  {
  buffer = 0;
  #ifdef USEROS
    tft.fillScreen(TFT_BLACK);
  #else
    tft.fillScreen(TFT_WHITE);
  #endif
  tft.setCursor(0, 0);
}

void cleanSprite() {
  buffer = 0;
  #ifdef USEROS
    spr.fillScreen(TFT_BLACK);
  #else
    spr.fillScreen(TFT_WHITE);
  #endif
  spr.setCursor(0, 0);
}

void updateSpr(){
  spr.pushSprite(0, 0);
}

void screen_init() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_RED);
  delay(100);
  #ifdef USEROS
    tft.fillScreen(TFT_BLACK);
  #endif
  tft.setCursor(0, 0);
  tft.setTextSize(1);
  tft.println("Inition done");
  // Serial.println("Screen init");
}

void sprPrint(String text, bool ln) {
  if (buffer > bufferSize) {
    cleanScreen();
  }
  if (ln) {
    buffer++;
    spr.println(text);
  } else {
    spr.print(text);
  }
}

void tftPrint(String text, bool ln) {
  if (buffer > bufferSize) {
    cleanScreen();
  }
  if (ln) {
    buffer++;
    tft.println(text);
  } else {
    tft.print(text);
  }
}

void click() {
  switch (menu) {
    case IDLE:
      Serial.println("IDLE state");
      tftPrint("IDLE state", true);
      break;
    case ForK:
      Serial.println("FK action start");
      cleanScreen();
      tftPrint("FK action start", true);
      for (int i = 0; i < MOTOR_NUM; i++) {
        tftPrint((String)theta[i], true);
      }
      runMotors();
      break;
    case InvK:
      Serial.println("IK action start");
      tftPrint("IK action start", true);
      runMotors();
      break;
    case ROS:
      tftPrint("ROS action start", true);
      runMotors();
      break;
    default:
      break;
  }
}

void longPress() {
  Serial.println("Home");
  tftPrint("Home", true);
  Homing();
}

void encoder(bool direction) {
  if (direction) {
    WRIST++;
    Serial.println("WRIST Length = " + (String)WRIST);
    tftPrint("WRIST Length = " + (String)WRIST, true);
  } else {
    WRIST--;
    Serial.println("WRIST Length = " + (String)WRIST);
    tftPrint("WRIST Length = " + (String)WRIST, true);
  }
}

void switchIDLE(){
  menu = IDLE;
  Serial.println("IDLE state");
  cleanScreen();
  tftPrint("IDLE state", true);
};
void switchFK(){
  menu = ForK;
  Serial.println("FK state");
  cleanScreen();
  tftPrint("FK state", true);
};
void switchIK(){
  menu = InvK;
  Serial.println("IK state");
  cleanScreen();
  tftPrint("IK state", true);
};
void switchROS(){
  menu = ROS;
  Serial.println("ROS state");
  cleanScreen();
  tftPrint("ROS state", true);
};
