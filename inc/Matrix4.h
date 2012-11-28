#ifndef MATRIX4_H
#define MATRIX4_H
#define _USE_MATH_DEFINES

#include <math.h>
#include "Vec4.h"
using namespace std;

class Matrix4{
private:
	float data[16];		// Data is stored in column major order
						// for the sake of openGL.
public:
	// Static Convenience Methods
	static Matrix4 identityM();
	static Matrix4 scaleM(float x, float y, float z);
	static Matrix4 translateM(float x, float y, float z);
	static Matrix4 rotateAxisM(char axis, float theta);

	// Constructors
    Matrix4();
	Matrix4(float* values);
	~Matrix4();
	Matrix4(const Matrix4& other);
	
	// Instance Methods
	// Operator Overloads
	Matrix4& operator=(const Matrix4& rhs);
	float& operator[](int index);
	const float& operator[](int index) const;
    float* get();
	float& get(int row, int col);
	Matrix4& operator+=(const Matrix4& rhs);
	Matrix4 operator+(const Matrix4& rhs);
	Matrix4& operator*=(const Matrix4& rhs);
	Matrix4 operator*(const Matrix4& rhs);
	Vec4 operator*(const Vec4& rhs);
	Matrix4& operator*=(const float scalar);
	Matrix4 operator*(const float scalar);

	// Debug Functions
	void print();
};
#endif