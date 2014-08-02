#include <inttypes.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include "geometry.h"

using namespace std;

ostream& operator<< (ostream& output, const Point3D& p)
{
	output << "(" << p.x << ", " << p.y << ", " << p.z << ")";
	return output; 
}

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

float Vector3D::getX()
{
	return x;
}

float Vector3D::getY()
{
	return y;
}

float Vector3D::getZ()
{
	return z;
}

float Vector3D::getLength()
{
	return length;
}

void Vector3D::normalize()
{
	x /= length;
	y /= length;
	z /= length;
	
	length = sqrt(x*x+y*y+z*z); 
} 

Line3D::Line3D(Point3D _startPoint, Point3D _endPoint) : 
	startPoint(_startPoint), endPoint(_endPoint), direction(startPoint, endPoint)
{
	length = direction.getLength();
	direction.normalize();
}

Line3D::Line3D(Point3D _startPoint, Vector3D _direction) : 
	startPoint(_startPoint), direction(_direction), endPoint()
{
	endPoint.x = startPoint.x + direction.getX();
	endPoint.y = startPoint.y + direction.getY();
	endPoint.z = startPoint.z + direction.getZ();
	length = direction.getLength();
}

Point3D Line3D::getStartPoint()
{
	return startPoint;
}

Point3D Line3D::getEndPoint()
{
	return endPoint;
}

float Line3D::getLength()
{
	return length;
}

Vector3D Line3D::getDirection()
{
	return direction;
}
