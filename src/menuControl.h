#ifndef MENU_CONTROL_H
#define MENU_CONTROL_H
#include "util.h"

// screen
void screen_init();
void tftPrint(String text, bool ln);
void sprPrint(String text, bool ln);
void cleanScreen();
void cleanSprite();
void updateSpr();

void click();
void longPress();
void encoder(bool direction);

// switch states
void switchIDLE();
void switchFK();
void switchIK();
void switchROS();

#endif // MENU_CONTROL_H
