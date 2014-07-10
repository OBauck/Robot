
#include "manipulator.h"
#include "servo.h"
#include "PCA9685.h"
#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

using namespace std;

int main()
{
	PCA9685 p(0x40, 50);

	float angle;
	int outNr;

	Servo servo1(&p, 720, 2340, -80, 80, 0);
	Servo servo2(&p, 720, 2340, -80, 90, 1);
	Servo servo3(&p, 720, 2340, -60, 90, 2);
	vector<float> q;
	servo1.setAngleDegree(0);
	servo2.setAngleDegree(0);
	servo3.setAngleDegree(0);
	
	while(1)
	{
		
		cout<<"enter servo"<<endl;
		cin>>outNr;
		cout<<"enter an angle"<<endl;
		cin>>angle;
		switch (outNr)
		{
			case 1:
				q = trajectoryPlanningDegrees(servo1.getAngleDegree(), angle);
				break;
			case 2:
				q = trajectoryPlanningDegrees(servo2.getAngleDegree(), angle);
				break;
			case 3:
				q = trajectoryPlanningDegrees(servo3.getAngleDegree(), angle);
				break;
		}
		
		
		vector<float>::iterator it;
		for (it = q.begin(); it != q.end(); it++)
		{
			switch (outNr)
			{
				case 1:
					servo1.setAngleDegree(*it);
					break;
				case 2:
					servo2.setAngleDegree(*it);
					break;
				case 3:
					servo3.setAngleDegree(*it);
					break;
			}	
			usleep(20000);
		}
	}
	
	return 0;
}
