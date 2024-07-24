#include <Arduino.h>
#include "util.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Setup Start");
  tftPrint("Setup Start", true);
  screen_init();
  #ifdef USEROS
    ros_init();
  #endif
  util_init();
  tasks_init();
  // Serial.println("Setup Done");
  tftPrint("Setup Done", true);
}

void loop() {
  vTaskDelay(1000);
}
