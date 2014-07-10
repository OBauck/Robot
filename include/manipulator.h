
#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <vector>
#include "servo.h"

using namespace std;

#define MAX_VELOCITY_DEGREE		120.0f		//degrees pr sec
#define MAX_VELOCITY_RAD		2.0944f	//MAX_VELOCITY_DEGREE*M_PI/180.0f	//rad pr sec
#define STEP_SIZE	0.02	//20ms

void trajectoryPlanningDegrees(Servo*, float, float);
vector<float> trajectoryPlanningDegrees(float, float);
vector<float> trajectoryPlanning(float, float, float);

class Manipulator
{
public:
Manipulator(PCA9685*, int, int, int);
void goToPosition(int, int, int);
void goToPositionSmooth(int, int, int);
void updatePosition(float, float, float);
void updatePositionSmooth(float, float, float);

private:
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo5;
Servo servo6;
Servo gripper;

float theta1;
float theta2;
float theta3;
float theta5;
float theta6;

int A1;
int A2;
int D6;
};

#endif
