
#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <iostream>

using namespace std;

class Point3D
{
public:
	Point3D(float thisx, float thisy, float thisz) : x(thisx), y(thisy), z(thisz) {};
	Point3D() {x = 0; y = 0; z = 0;};
	float x;
	float y;
	float z;
	friend ostream& operator<< (ostream& output, const Point3D& p);
};

class Vector3D
{
public:
	Vector3D(float, float, float);
	Vector3D(Point3D,Point3D);
	void normalize();
	float getLength();
	float getX();
	float getY();
	float getZ();
private:
	float x;
	float y;
	float z;
	float length;
};

class Circle3D
{
public:
	Circle3D(Point3D, float, float);
private:
	Point3D center;
	float radius;
	float theta;
	Point3D startPoint;
	Point3D endPoint;
	Vector3D normal;
	float length;
};

class Line3D
{
public:
	Line3D(Point3D, Point3D);
	Line3D(Point3D, Vector3D);
	float getLength();
	Point3D getStartPoint();
	Point3D getEndPoint();
	Vector3D getDirection();
private:
	Point3D startPoint;
	Point3D endPoint;
	Vector3D direction;
	float length;
};


#endif
