
#include "manipulator.h"
#include "servo.h"
#include "PCA9685.h"
#include "geometry.h"
#include "interface.h"
#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <vector>

using namespace std;

int main()
{
	PCA9685 p(0x40, 50);
	
	int x, y, z;
	Point3D position;
	Manipulator man(&p, 99, 104, 0);

	Point3D startPoint(-50,150,0);
	Point3D endPoint(50, 150, 0);
	
	Line3D line(startPoint, endPoint);

	cin>>x;

	man.followLine(line);
	
//	windowInit();
	
	while(1)
	{
//		checkEvent(&man);
//		cout<<"x: ";
//		cin>>x;
//		cout<<"y: ";
//		cin>>y;
//		cout<<"z: ";
//		cin>>z;

//		position = Point3D(x,y,z);

//		man.goToPositionSmoothQuintic(position);
	}
	
	return 0;
}
