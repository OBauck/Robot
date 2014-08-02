
#include "manipulator.h"
#include "servo.h"
#include "PCA9685.h"
#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

using namespace std;

struct posAngles 
{
	float theta1;
	float theta2;
	float theta3;
};

/*
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
}*/

vector<float> trajectoryPlanningQuintic(float startAngle, float endAngle, float tf)
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

vector<float> trajectoryPlanningLSPD(float startAngle, float endAngle, float tf)
{
	float t1, t2 ,q0, a2, a3, angle, v, q1;
	
	v = MAX_VELOCITY_RAD*sgn(endAngle-startAngle);
	
	t1 = (startAngle - endAngle + v*tf)/v;
	t2 = tf - t1;
	q0 = startAngle;
	a2 = 3*v/pow(t1,2);
	a3 = -2*v/pow(t1,3);
	q1 = (endAngle + startAngle - v*tf)/2;
	
	int n;
	n = tf/STEP_SIZE;
	float t = 0;
	vector<float> output;
	for (int i=0; i<n; i++)
	{
		if (t < t1)
			angle = q0 + a2/3*pow(t,3) + a3/4*pow(t,4);
		else if (t < t2)
			angle = q1 + v*t;
		else
			angle = q1 + v*t2 + v*(t-t2) - a2/3*pow(t-t2,3) - a3/4*pow(t-t2,4);
		
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

void Manipulator::goToPositionPencil(Point3D p)
{
	float c3, s3, t1, t2, t3, t5;
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
	
	t5 = M_PI - t2 - t3;
	
	updatePosition(t1, t2, t3);
	updateOrientation(t5);
}

void Manipulator::goToPositionSmoothQuintic(Point3D p)
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
	
	updatePositionSmoothQuintic(t1, t2, t3);
}

void Manipulator::goToPositionSmoothLSPD(Point3D p)
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
	
	updatePositionSmoothLSPD(t1, t2, t3);
}

void Manipulator::updateOrientation(float t5, float t6)
{
	theta5 = t5;
	theta6 = t6;
	
	servo5.setAngleRad(theta5);
	servo6.setAngleRad(theta6);
}

void Manipulator::updateOrientation(float t5)
{
	theta5 = t5;
	servo5.setAngleRad(theta5);
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

void Manipulator::updatePositionSmoothQuintic(float t1, float t2, float t3)
{
	vector<float> q1, q2, q3;
	float tf1, tf2, tf3, tf;
	tf1 = 30.0/16.0*fabs(t1 - theta1)/MAX_VELOCITY_RAD;
	tf2 = 30.0/16.0*fabs(t2 - theta2)/MAX_VELOCITY_RAD;
	tf3 = 30.0/16.0*fabs(t3 - theta3)/MAX_VELOCITY_RAD;

	tf = fmax(tf1, tf2);
	tf = fmax(tf3, tf);

	cout<<"tf: "<<tf<<endl;
	q1 = trajectoryPlanningQuintic(theta1, t1, tf);
	q2 = trajectoryPlanningQuintic(theta2, t2, tf);
	q3 = trajectoryPlanningQuintic(theta3, t3, tf);
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

void Manipulator::updatePositionSmoothLSPD(float t1, float t2, float t3)
{
	vector<float> q1, q2, q3;
	float tf1, tf2, tf3, tf;
	tf1 = 4.0/3.0*fabs(t1 - theta1)/MAX_VELOCITY_RAD;
	tf2 = 4.0/3.0*fabs(t2 - theta2)/MAX_VELOCITY_RAD;
	tf3 = 4.0/3.0*fabs(t3 - theta3)/MAX_VELOCITY_RAD;

	tf = fmax(tf1, tf2);
	tf = fmax(tf3, tf);

	cout<<"tf: "<<tf<<endl;
	q1 = trajectoryPlanningLSPD(theta1, t1, tf);
	q2 = trajectoryPlanningLSPD(theta2, t2, tf);
	q3 = trajectoryPlanningLSPD(theta3, t3, tf);
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

void Manipulator::followLine(Line3D line)
{
	float steps = line.getLength()/STEP_SIZE_CURVE;
	
	for (int i = 0; i < steps; i++)
	{
		goToPositionPencil(Point3D((line.getStartPoint()).x + i*(line.getDirection()).getX(), (line.getStartPoint()).y + i*(line.getDirection()).getY(),
		(line.getStartPoint()).z + i*(line.getDirection()).getZ()));
	}
}
