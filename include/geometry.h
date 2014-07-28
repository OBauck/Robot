
#ifndef GEOMETRY_H_
#define GEOMETRY_H_

using namespace std;

class Point3D
{
public:
	Point3D(float thisx, float thisy, float thisz) : x(thisx), y(thisy), z(thisz) {};
	Point3D() {x = 0; y = 0; z = 0;};
	float x;
	float y;
	float z;
};

class Vector3D
{
public:
	Vector3D(float, float, float);
	Vector3D(Point3D,Point3D);
	void normalize();
	float length;
	float getX();
	float getY();
	float getZ();
private:
	float x;
	float y;
	float z;
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

class line3D
{
public:
	line3D(Point3D, Point3D);
	line3D(Point3D, Vector3D);
private:
	Point3D startPoint;
	Point3D endPoint;
	Vector3D direction;
	float length;
};


#endif
