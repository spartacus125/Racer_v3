//	Vec4.cpp
//	Author: Morgan Strong
//	10/30/12
//	
//	Intended for use in 3D graphics.

#include "../inc/Vec4.h"
using namespace std;

// Constructors
Vec4::Vec4(){
	memset(data,0,sizeof(float)*4);
}
Vec4::Vec4(float x, float y, float z, float w){
	data[0]=x;
	data[1]=y;
	data[2]=z;
	data[3]=w;
}
Vec4::Vec4(float* values){
	memcpy(data,values,sizeof(float)*4);
}
Vec4::Vec4(const Vec4& other ){
	memcpy(data,other.data,sizeof(float)*4);
}
Vec4::~Vec4(){
	// Nothing Stored On Heap
}

// Operator Overloads
float& Vec4::operator[](int index){
	return data[index];
}
const float& Vec4::operator[](int index) const{
	return data[index];
}
Vec4& Vec4::operator=(Vec4& rhs){
	if(this != &rhs){
		memcpy(data, rhs.data,sizeof(float)*4);
	}
	return *this;
}
Vec4& Vec4::operator+=(const Vec4& rhs){
	for(int i=0; i<4; i++)
		data[i]+=rhs[i];
	return *this;
}
Vec4 Vec4::operator+(const Vec4& rhs){
	Vec4 ret = *this;
	ret += rhs;
	return ret;
}
Vec4& Vec4::operator-=(const Vec4& rhs){
	for(int i=0; i<4; i++)
		data[i]-=rhs[i];
	return *this;
}
Vec4 Vec4::operator-(const Vec4& rhs){
	Vec4 ret = *this;
	ret -= rhs;
	return ret;
}
Vec4& Vec4::operator*=(const float scalar){
	for(int i=0;i<4;i++)
		data[i]*=scalar;
	return *this;
}
Vec4 Vec4::operator*(const float scalar){
	Vec4 ret = *this;
	ret *= scalar;
	return ret;
}
Vec4& Vec4::operator/=(const float scalar){
	for(int i=0;i<4;i++)
		data[i]/=scalar;
	return *this;
}
Vec4 Vec4::operator/(const float scalar){
	Vec4 ret = *this;
	ret /= scalar;
	return ret;
}
Vec4& Vec4::operator*=(const Vec4& rhs){
	for(int i=0;i<4;i++)
		data[i]*=rhs[i];
	return *this;
}
Vec4 Vec4::operator*(const Vec4& rhs){
	Vec4 ret = *this;
	ret *= rhs;
	return ret;
}
float Vec4::distance(Vec4 p2){
	float dist = 0;
	// sum squares
	for(int i=0; i<3; i++)
		dist += pow((p2[i]-data[i]),2.0f);
	return pow(dist,.5f);
}
float Vec4::length(){
	float len = 0;
	// sum squares
	for(int i=0; i<3; i++)
		len += pow(data[i],2.0f);
	return pow(len,.5f);
}
void Vec4::normalize(){
	float len = length();
	for(int i=0; i<3; i++)
		data[i] /= len;
}
Vec4 Vec4::normalized(){
	Vec4 ret = *this;
	ret.normalize();
	return ret;
}
void Vec4::clamp(float min_l, float max_l){
	for(int i=0;i<4;i++)
		data[i] = max(min_l, min(max_l, data[i]));
}
Vec4 Vec4::clamped(float min_l, float max_l){
	Vec4 ret = *this;
	ret.clamp();
	return ret;
}
Vec4 Vec4::cross(Vec4 rhs){
	Vec4 ret;
	ret[0] = (data[1] * rhs[2]) - (data[2] * rhs[1]);
	ret[1] = (data[2] * rhs[0]) - (data[0] * rhs[2]);
	ret[2] = (data[0] * rhs[1]) - (data[1] * rhs[0]);
	ret[3] = 1;
	return ret;
}
float Vec4::dot(Vec4 rhs){
	float dot = 0;
	// Sum Products
	for(int i=0; i<3; i++)
		dot += data[i]*rhs[i];
	return dot;
}
void Vec4::print(){
	cout.precision(3);
	for(int i=0; i<4; i++){
		cout << setw(10) << fixed << data[i];
	}
	cout << endl;
}