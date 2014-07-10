
#include "manipulator.h"
#include "servo.h"
#include "PCA9685.h"
#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

using namespace std;

void trajectoryPlanningDegrees(Servo *servo, float startAngle, float endAngle)
{
	float tf, a0, a3, a4, a5, angle;
	tf = 30.0/16.0*(endAngle-startAngle)/MAX_VELOCITY_DEGREE;
	a0 = startAngle;
	a3 = 10*(endAngle-startAngle)/pow(tf,3);
	a4 = -15*(endAngle-startAngle)/pow(tf,4);
	a5 = 6*(endAngle-startAngle)/pow(tf,5);
	
	int n = tf/STEP_SIZE;
	float t = 0;
	for (int i=0; i<n; i++)
	{
		angle = a0 + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
		servo->setAngleDegree(angle);
		t += STEP_SIZE;
		usleep(20000);
	}
	
}

vector<float> trajectoryPlanningDegrees(float startAngle, float endAngle)
{
	float tf, a0, a3, a4, a5, angle;
	tf = 30.0/16.0*fabs(endAngle-startAngle)/MAX_VELOCITY_DEGREE;
	a0 = startAngle;
	a3 = 10*(endAngle-startAngle)/pow(tf,3);
	a4 = -15*(endAngle-startAngle)/pow(tf,4);
	a5 = 6*(endAngle-startAngle)/pow(tf,5);

	int n;
	n = tf/STEP_SIZE;
	float t = 0;
	vector<float> output;
	for (int i=0; i<n; i++)
	{
		angle = a0 + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
		output.push_back(angle);
		t += STEP_SIZE;
	}

	return output;
}

vector<float> trajectoryPlanning(float startAngle, float endAngle, float tf)
{
	float a0, a3, a4, a5, angle;
	a0 = startAngle;
	a3 = 10*(endAngle-startAngle)/pow(tf,3);
	a4 = -15*(endAngle-startAngle)/pow(tf,4);
	a5 = 6*(endAngle-startAngle)/pow(tf,5);

	int n;
	n = tf/STEP_SIZE;
	float t = 0;
	vector<float> output;
	for (int i=0; i<n; i++)
	{
		angle = a0 + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
		output.push_back(angle);
		t += STEP_SIZE;
	}

	return output;
}

Manipulator::Manipulator(PCA9685* pca, int this_A1, int this_A2, int this_D6) : 
	servo1(pca, 720, 2340, -80, 80, 0),
	servo2(pca, 720, 2340, -80, 90, 1),
	servo3(pca, 720, 2340, -60, 90, 2),
	servo5(pca, 720, 2340, -60, 90, 3),
	servo6(pca, 720, 2340, -60, 90, 4),
	gripper(pca, 720, 2340, -60, 90, 5)
{
	
	updatePosition(0,0,0);
	theta5 = 0;
	theta6 = 0;
	
	A1 = this_A1;
	A2 = this_A2;
	D6 = this_D6;
}

void Manipulator::goToPosition(Point3D p)
{
	float c3, s3, t1, t2, t3;
	c3 = (p.z*p.z+p.x*p.x+p.y*p.y-A1*A1-A2*A2)/(2.0*A1*A2);
	s3 = sqrt(1-c3*c3);
	if(s3 != s3)
	{
		cout<<"Position out of reach, setting sin(theta3) to zero"<<endl;
		s3 = 0;
	}
	t3 = atan2(s3, c3);
	t2 = M_PI/2.0 - atan2(p.z, sqrt(p.x*p.x + p.y*p.y)) - atan2(A2*s3, A1 + A2*c3);
	t1 = atan2(-p.x, p.y);
	
	updatePosition(t1, t2, t3);
}

void Manipulator::goToPositionSmooth(int x, int y, int z)
{
	float c3, s3, t1, t2, t3;
	c3 = (p.z*p.z+p.x*p.x+p.y*p.y-A1*A1-A2*A2)/(2.0*A1*A2);
	s3 = sqrt(1-c3*c3);
	if(s3 != s3)
	{
		cout<<"Position out of reach, setting sin(theta3) to zero"<<endl;
		s3 = 0;
	}
	t3 = atan2(s3, c3);
	t2 = M_PI/2.0 - atan2(p.z, sqrt(p.x*p.x + p.y*p.y)) - atan2(A2*s3, A1 + A2*c3);
	t1 = atan2(-p.x, p.y);
	
	updatePositionSmooth(t1, t2, t3);
}

void Manipulator::updatePosition(float t1, float t2, float t3)
{
	theta1 = t1;
	theta2 = t2;
	theta3 = t3;
	
	servo1.setAngleRad(theta1);
	servo2.setAngleRad(theta2);
	servo3.setAngleRad(theta3);
}

void Manipulator::updatePositionSmooth(float t1, float t2, float t3)
{
	vector<float> q1, q2, q3;
	float tf1, tf2, tf3, tf;
	tf1 = 30.0/16.0*fabs(t1 - theta1)/MAX_VELOCITY_RAD;
	tf2 = 30.0/16.0*fabs(t2 - theta2)/MAX_VELOCITY_RAD;
	tf3 = 30.0/16.0*fabs(t3 - theta3)/MAX_VELOCITY_RAD;

	tf = fmax(tf1, tf2);
	tf = fmax(tf3, tf);

	cout<<"tf: "<<tf<<endl;
	q1 = trajectoryPlanning(theta1, t1, tf);
	q2 = trajectoryPlanning(theta2, t2, tf);
	q3 = trajectoryPlanning(theta3, t3, tf);
	if ( q1.size() != q2.size() || q1.size() != q2.size() )
	{
		cout<<"vectors not the same length!"<<endl;
		return;
	}
	
	int size = q1.size();
	for (int i = 0; i < size; i++)
	{
		servo1.setAngleRad(q1[i]);
		servo2.setAngleRad(q2[i]);
		servo3.setAngleRad(q3[i]);
		usleep(20000);
	}

	theta1 = t1;
	theta2 = t2;
	theta3 = t3;
	
}
