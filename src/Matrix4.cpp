//	Matrix4.cpp
//	Author: Morgan Strong
//	10/30/12
//	
//	Intended for use with 3D Graphics.

#include "../inc/Matrix4.h"
using namespace std;

// Static Convenience Methods
Matrix4 Matrix4::identityM(){
	return Matrix4();
}
Matrix4 Matrix4::scaleM(float x, float y, float z){
	float i[16]={x,0,0,0,0,y,0,0,0,0,z,0,0,0,0,1};
	return Matrix4(i);
}
Matrix4 Matrix4::translateM(float x, float y, float z){
	float i[16]={1,0,0,0,0,1,0,0,0,0,1,0,x,y,z,1};
	return Matrix4(i);
}
Matrix4 Matrix4::rotateAxisM(char axis, float theta){
	switch(axis){
	case 'x':{
		float i[16] = {1,0,0,0,0,cos(theta),sin(theta),0,0,-sin(theta),cos(theta),0,0,0,0,1};
		return Matrix4(i);}
	case 'y':{
		float i[16] = {cos(theta),0,-sin(theta),0,0,1,0,0,sin(theta),0,cos(theta),0,0,0,0,1};
		return Matrix4(i);}
	case 'z':{
		float i[16] = {cos(theta),sin(theta),0,0,-sin(theta),cos(theta),0,0,0,0,1,0,0,0,0,1};
		return Matrix4(i);}
	default:
		// Don't be stupid.
		return identityM();
	}
}

// Constructors
Matrix4::Matrix4() {
    float i[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    memcpy(data, i, sizeof(float)*16);
}
Matrix4::Matrix4(float* values){
	memcpy(data, values, sizeof(float)*16); // Column Major Order
}
Matrix4::~Matrix4(){
	// Nothing on the Heap.
}
Matrix4::Matrix4(const Matrix4& other){
	memcpy(data, other.data,sizeof(float)*16);
}

// Instance Methods
// Operator Overloads
Matrix4& Matrix4::operator=(const Matrix4& rhs){
	if(this != &rhs){
		memcpy(data, rhs.data,sizeof(float)*16);
	}
	return *this;
}
float& Matrix4::operator[](int index){
	return data[index]; //non-const
}
const float& Matrix4::operator[](int index) const{
	return data[index]; //const
}
float* Matrix4::get() {
    return data;
}
float& Matrix4::get(int row, int col){
	return data[row+col*4];
}
Matrix4& Matrix4::operator+=(const Matrix4& rhs){
	for(int i=0; i<16; i++)
		data[i]+=rhs[i];
	return *this;
}
Matrix4 Matrix4::operator+(const Matrix4& rhs){
	Matrix4 ret = *this;
	ret += rhs;
	return ret;
}
Matrix4& Matrix4::operator*=(const Matrix4& rhs){
	float new_data[16];
	for(int i=0; i<16; i++){
		int row = i % 4;
		int col = i - row;
		float acc = 0;
		acc += data[0+row]*rhs[col+0];
		acc += data[4+row]*rhs[col+1];
		acc += data[8+row]*rhs[col+2];
		acc += data[12+row]*rhs[col+3];
		new_data[i]=acc;
	}
	memcpy(data,new_data,sizeof(float)*16);
	return *this;
}
Matrix4 Matrix4::operator*(const Matrix4& rhs){
	Matrix4 ret = *this;
	ret *= rhs;
	return ret;
}
Vec4 Matrix4::operator*(const Vec4& rhs){
	float new_data[4];
	for(int row=0;row<4;row++)
		new_data[row] = data[0+row]*rhs[0]+
						data[4+row]*rhs[1]+
						data[8+row]*rhs[2]+
						data[12+row]*rhs[3];
	return Vec4(new_data);
}
Matrix4& Matrix4::operator*=(const float scalar){
	for(int i=0;i<16;i++)
		data[i]*=scalar;
	return *this;
}
Matrix4 Matrix4::operator*(const float scalar){
	Matrix4 ret = *this;
	ret *= scalar;
	return ret;
}

// Debug Functions
void Matrix4::print(){
	cout.precision(3);
	for(int row=0; row<4; row++){
		cout << setw(10) << fixed << data[0+row];
		cout << setw(10) << fixed << data[4+row];
		cout << setw(10) << fixed << data[8+row];
		cout << setw(10) << fixed << data[12+row];
		cout << endl;
	}
}