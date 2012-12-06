#ifndef VEC4_H
#define VEC4_H

#include <iostream>
#include <iomanip>
#include <string>
using namespace std;

class Vec4{
private:
	float data[4];
public:
	// Constructors
	Vec4();
	Vec4(float x, float y, float z, float w=1);
	Vec4(float* values);
	~Vec4();
	Vec4(const Vec4& other );

	//Operator Overloads
	float& operator[](int index);
	const float& operator[](int index) const;
	Vec4& operator=(Vec4& rhs);
	Vec4& operator+=(const Vec4& rhs);
	Vec4 operator+(const Vec4& rhs);
	Vec4& operator-=(const Vec4& rhs);
	Vec4 operator-(const Vec4& rhs);
	Vec4& operator*=(const float scalar);
	Vec4 operator*(const float scalar);
	Vec4& operator/=(const float scalar);
	Vec4 operator/(const float scalar);
	Vec4& operator*=(const Vec4& rhs);		// Component-wise *
	Vec4 operator*(const Vec4& rhs);		// Component-wise *
	
	//Convenience Functions
	float distance(Vec4 p2);
	float length();
	void normalize();
	Vec4 normalized();
	void clamp(float min=0, float max=1);
	Vec4 clamped(float min=0, float max=1);
	Vec4 cross(Vec4 rhs);
	float dot(Vec4 rhs);
	void print();
};

#endif