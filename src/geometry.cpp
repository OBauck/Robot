#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

using namespace std;

Vector3D::Vector3D(Point3D startPoint,Point3D endPoint)
{
	x = endPoint.x - startPoint.x;
	y = endPoint.y - startPoint.y;
	z = endPoint.z - startPoint.z;
	length = sqrt(x*x+y*y+z*z);
}

Vector3D::Vector3D(float thisx, float thisy, float thisz)
{
	x = thisx;
	y = thisy;
	z = thisz;
	length = sqrt(x*x+y*y+z*z);
}

void Vector3D::normalize()
{
	x /= length;
	y /= length;
	z /= length;
	
	length = sqrt(x*x+y*y+z*z); 
} 

line3D::line3D(Point3D _startPoint, Point3D _endPoint) : 
	startPoint(_startPoint), endPoint(_endPoint), direction(startPoint, endPoint)
{
	length = direction.length;
}

line3D::line3D(Point3D _startPoint, Vector3D _direction) : 
	startPoint(_startPoint), Vector3D(direction), endPoint()
{
	endPoint.x = startPoint.x + direction.x;
	endPoint.y = startPoint.y + direction.y;
	endPoint.z = startPoint.z + direction.z;
	length = direction.length;
}