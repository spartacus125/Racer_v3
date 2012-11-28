//	Class:		Geometry
//	Author:		Morgan Strong
//	Updated:	11/15/12
//	Project:	CS 455 Game - Racer
//
//	Representative of a Non-Deformable 3D object. Generally a container class
//	with built-in space for dynamic transforms (physics simulation, user input,
//	etc.)  Color, Texture, and other data should be stored outside this class.
//	This class is built generally, but particularly friendly to openGL.

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <map>
#include <string>
#include <vector>
#include "Vec4.h"
#include "Matrix4.h"
#include "KineticState.h"
#include "Face.h"
#include "Matrix4.h"
using namespace std;

class Geometry{
private:
	// Members
	string id;
	vector<Vec4> vertices;
	vector<Vec4> normals;
	vector<Vec4> uvs;
	vector<Face> faces;
	map<int,vector<Vec4*> > point_groups;
	Matrix4 scale;
	Matrix4 rotate;
	Matrix4 translate;
	KineticState dynamic_transforms;
	Geometry* parent;

public:
	// Constructors
	Geometry();
	Geometry(const Geometry& other);
	~Geometry();

	Geometry(string new_id,
		int num_v = 0, float** vertices = NULL,
		int num_n = 0, float** normals = NULL,
		int num_u = 0, float** uvs = NULL);

	Geometry(string new_id,
		vector<Vec4> _verts,
		vector<Vec4> _norms,
		vector<Vec4> _uvs,
		vector<Face> _faces,
		Geometry* parent);

	
	// Instance Methods
	string GetId();
	void Scale(float x, float y, float z);
	void Rotate(float x, float y, float z);
	void Translate(float x, float y, float z);
	void GenerateTransform();
	void Draw();

};

#endif