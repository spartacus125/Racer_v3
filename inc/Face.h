//	Class:		Face
//	Updated:	11/18/12
//	Project:	CS 455 Game - Racer
//
//  Represents a geometric face

#ifndef FACE_H
#define FACE_H

#include <iostream>
#include <vector>

#include "Vec4.h"

using namespace std;

class Face
{
public:
	vector<Vec4> vertices;
	vector<Vec4> uvs;
	vector<Vec4> normals;
};

#endif