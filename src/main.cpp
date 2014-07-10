
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

	int x, y, z;
	Manipulator man(&p, 99, 104, 0);

	
	while(1)
	{
		cout<<"x: ";
		cin>>x;
		cout<<"y: ";
		cin>>y;
		cout<<"z: ";
		cin>>z;

		man.goToPositionSmooth(x, y, z);
	}
	
	return 0;
}
