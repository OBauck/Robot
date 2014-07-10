
#ifndef SERVO_H_
#define SERVO_H_

#include "PCA9685.h"

#define SERVOFREQUENCY 50	//Hz
#define SERVOCYCLELENGTH_US	20000	//20ms

class Servo
{
	public:
	Servo(PCA9685*, int, int, int, int, int);
	void setAngleRad(float);
	void setAngleDegree(float);
	float getAngleDegree() {return(angle_degree); };
	float getAngleRad() {return(angle_rad); };
		
	private:
	PCA9685 *pca;
	int out_nr;
	
	int duty_low;
	int duty_high;
	float degree_low;
	float degree_high;
	
	float a;
	float b;	//a*degree + b = on_value
	
	float rad_low;
	float rad_high;
	float angle_rad;
	float angle_degree;
};

#endif
