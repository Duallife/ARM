#include "serial.h"

void serialTask(){
  if (Serial.available() > 0) {

    char inputString[maxLength];  // Array to hold the input string
    char word[10];                // Array to hold the first word (adjust size as needed)
    double inputPose[maxElements];   // Array to hold the parsed double values
    int valueCount = 0;  

    int len = Serial.readBytesUntil('\n', inputString, maxLength - 1);
    inputString[len] = '\0';  // Null-terminate the string

    // Split the string by commas
    char* token = strtok(inputString, ",");
    int tokenIndex = 0;
    valueCount = 0;

    while (token != NULL) {
      if (tokenIndex == 0) {
        // Store the first token (word) in the word array
        strncpy(word, token, sizeof(word) - 1);
        word[sizeof(word) - 1] = '\0';  // Ensure the word is null-terminated
      } else {
        // Convert subsequent tokens to double and store in values array
        if (valueCount < maxElements) {
          inputPose[valueCount] = atof(token);  // Convert token to double
          valueCount++;
        }
      }
      token = strtok(NULL, ",");
      tokenIndex++;
    }
    

    if (strcmp(word, "FK") == 0) { switchFK();}
    else if (strcmp(word, "IK") == 0) { switchIK();}
    else if (strcmp(word, "ROS") == 0) { switchROS();}
    else if (strcmp(word, "IDLE") == 0) { switchIDLE();}
    else if (strcmp(word, "POS") == 0){ switchIDLE();getCurrentPosition();}
    else if (strcmp(word, "RUN") == 0){ runMotors();}
    else if (strcmp(word, "STOP") == 0){ forceStop();}
    
    
    switch (menu) {
      case IDLE:
        Serial.println("IDLE state");
        break;

      case ForK:
        Serial.println("FK state");
        FK(inputPose);
        printTarget();
        break;

      case InvK:{
        Serial.println("IK state");
        double tmp[3] = {inputPose[0], inputPose[1], inputPose[2]};
        double rotTemp[3][3];
        eul2rotm(tmp, rotTemp);
        double result[4][4] = {
          {rotTemp[0][0], rotTemp[0][1], rotTemp[0][2], inputPose[3]},
          {rotTemp[1][0], rotTemp[1][1], rotTemp[1][2], inputPose[4]},
          {rotTemp[2][0], rotTemp[2][1], rotTemp[2][2], inputPose[5]},
          {0, 0, 0, 1}};
        setTarget(result, target);
        IK(target);
        printTarget();
        break;}

      case ROS:
        Serial.println("ROS state");
        tftPrint("ROS state", 1);
        break;
      default:
        break;
    }
     
    memset(inputString, 0, sizeof(inputString));
  }
  vTaskDelay(5);
}