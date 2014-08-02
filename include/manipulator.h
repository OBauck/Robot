
#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <vector>
#include "servo.h"
#include "geometry.h"

using namespace std;

#define MAX_VELOCITY_DEGREE		120.0f		//degrees pr sec
#define MAX_VELOCITY_RAD		2.0944f	//MAX_VELOCITY_DEGREE*M_PI/180.0f	//rad pr sec
#define STEP_SIZE	0.02f	//20ms
#define STEP_SIZE_CURVE		1.0f	//1mm

#define XSTART 0
#define YSTART 0
#define ZSTART 203

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

struct posAngles 
{
	float theta1;
	float theta2;
	float theta3;

};


vector<float> trajectoryPlanningQuintic(float, float, float);
vector<float> trajectoryPlanningLSPD(float, float, float);
posAngles calculateAngles(Point3D, int, int);

class Manipulator
{
public:
Manipulator(PCA9685*, int, int, int);
void goToPosition(Point3D);
void goToPositionPencil(Point3D);
void goToPositionSmoothQuintic(Point3D);
void goToPositionSmoothLSPD(Point3D);

void updateOrientation(float);
void updateOrientation(float, float);

void updatePosition(posAngles);
void updatePositionSmoothQuintic(posAngles);
void updatePositionSmoothLSPD(posAngles);

void followLine(Line3D);

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
