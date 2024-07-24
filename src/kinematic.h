#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <Arduino.h>
#include "matrix.h"
#include <math.h>
#include "util.h"

// #define PI 3.14159265358979323846
extern double WRIST;

void FK(double inputPose[6]);
void IK(double target[4][4]);

void rotm2eul(double R[3][3], double euler[3]);
void eul2rotm(double euler[3], double R[3][3]);
void printMatrix(double* T, int r, int c);

#endif // KINEMATIC_H
