
#include "servo.h"
#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

using namespace std;

Servo::Servo(PCA9685 *this_pca, int us_low, int us_high, int deg_low, int deg_high, int this_out_nr)
{
	pca = this_pca;
	out_nr = this_out_nr;
	
	duty_high = (float)us_high/SERVOCYCLELENGTH_US*4095;
	duty_low = (float)us_low/SERVOCYCLELENGTH_US*4095;

	degree_high = deg_high;
	degree_low = deg_low;
	rad_high = degree_high/180*M_PI;
	rad_low = degree_low/180*M_PI;
	
	a = (duty_high-duty_low)/(rad_high-rad_low);
	b = (rad_high*duty_low - rad_low*duty_high)/(rad_high-rad_low);
}

void Servo::setAngleRad(float rad)
{
	if (rad < rad_low)
	{
		cout<<"angle too low, setting angle to minimum: "<<degree_low<<" degrees"<<endl;
		setAngleRad(rad_low);
		return;
	}
	if (rad > rad_high)
	{
		cout<<"angle too high, setting angle to maximum: "<<degree_high<<" degrees"<<endl;
		setAngleRad(rad_high);
		return;
	}
	
	float on_value = a*rad+b;
	pca->setPWM(out_nr, on_value);
	angle_rad = rad;
	angle_degree = rad/M_PI*180;
}

void Servo::setAngleDegree(float degree)
{
	setAngleRad(degree/180*M_PI);
}
