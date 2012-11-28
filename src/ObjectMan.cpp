//	Class:		ObjectMan
//	Updated:	11/18/12
//	Project:	CS 455 Game - Racer
//
//	Acts as manager for all objects within the game.

#define ENLISTMENTS "models/models.enlistments"
#define MODELSPATH "models/"
#define GEOEXT ".obj"
#define TEXEXT ".bmp"

#include <fstream>
#include <sstream>
#include "../inc/main.h"
#include "../inc/ObjectMan.h"
#include "../inc/TextureMan.h"
#include "../lib/kixor_obj/objLoader.h"

ObjectMan* ObjectMan::instance = NULL;

ObjectMan* ObjectMan::GetInstance() 
{
	if (instance == NULL)
	{
		instance = new ObjectMan();
	}
	return instance;
}

Geometry* ObjectMan::GetId(string id) {
	return GetInstance()->getId(id);
}

vector<Geometry*> ObjectMan::GetObjects() {
	return GetInstance()->getObjects();
}

vector<Geometry*> ObjectMan::GetCollidables() {
	return GetInstance()->getCollidables();
}

void ObjectMan::UpdateObjectsWithInput()
{
	Geometry* car = GetInstance()->objectMap[CAR];
	float z = InputMan::GetMovementBackward() - InputMan::GetMovementForward();
	float x = InputMan::GetMovementRight() - InputMan::GetMovementLeft();
	bool accel = InputMan::GetAccel();
	car->Translate(accel * x * TRANSLATE_SCALE, 0.0, accel * z * TRANSLATE_SCALE);
}

ObjectMan::ObjectMan()
{
	LoadObjects();
	cout << "Collidable Objects:" << endl;
	for (int i = 0; i < collidables.size(); i++)
	{
		cout << "\t" << collidables[i]->GetId() << endl;
	}
	cout << endl;
}

void ObjectMan::LoadObjects()
{
	ifstream enlistments(ENLISTMENTS);
	
	if(!enlistments.is_open())
	{
		cout << "Bad models.enlistments file:" << endl
			<< "\tMake sure you have a models.enlistments file in your models "
			<< "directory that contains the names of each object. Leave off the "
			<< "file extension. Textures should have the same name." << endl;
		return;
	}

	cout << "Loading Objects:" << endl;

	while(enlistments.good())
	{
		string line;
		getline(enlistments, line);

		if (line.find("#") == 0 || line.empty())
		{
			// Comment or empty line in enlistments - skip over it
			continue;
		}

		string obj_id;
		string filename;
		float tx, ty, tz, rx, ry, rz, sx, sy, sz;
		string parent_id;

		istringstream iss(line);
		iss >> obj_id;
		iss >> filename;
		iss >> tx;
		iss >> ty;
		iss >> tz;
		iss >> rx;
		iss >> ry;
		iss >> rz;
		iss >> sx;
		iss >> sy;
		iss >> sz;
		iss >> parent_id;

		bool collidable = false;
		if (filename.find("::") != string::npos)
		{
			collidable = true;
			filename = filename.substr(0, filename.find("::"));
		}

		Geometry* obj = LoadObject(obj_id, filename, collidable, parent_id);

		cout << "\t\tTransformations...";

		obj->Translate(tx, ty, tz);
		obj->Rotate(rx, ry, rz);
		obj->Scale(sx, sy, sz);

		cout << "DONE." << endl;

		LoadTexture(obj_id, filename);
	}

	cout << "Loading Complete." << endl << endl;
}

Geometry* ObjectMan::LoadObject(string id,
	string name,
	bool collidable,
	string parent_id)
{
	objLoader* loader = new objLoader();
	string filename = MODELSPATH + name + GEOEXT;
	
	cout << "\t" << filename << "...";
	
	char* file = (char*)(filename).c_str();
	loader->load(file);

	obj_vector** verts = loader->vertexList;
	int num_v = loader->vertexCount;
	vector<Vec4> vertices;

	obj_vector** norms = loader->normalList;
	int num_n = loader->normalCount;
	vector<Vec4> normals;

	obj_vector** uvs = loader->textureList;
	int num_u = loader->textureCount;
	vector<Vec4> uv_points;

	obj_face** face_list = loader->faceList;
	int num_f = loader->faceCount;
	vector<Face> faces;

	for (int i = 0; i < num_v; i++)
	{
		obj_vector* cur = verts[i];
		Vec4 point((float)cur->e[0], (float)cur->e[1], (float)cur->e[2]);
		vertices.push_back(point);
	}

	for (int j = 0; j < num_n; j++)
	{
		obj_vector* cur = norms[j];
		Vec4 norm((float)cur->e[0], (float)cur->e[1], (float)cur->e[2]);
		normals.push_back(norm);
	}

	for (int k = 0; k < num_u; k++)
	{
		obj_vector* cur = uvs[k];
		Vec4 uv_point((float)cur->e[0], (float)cur->e[1], 0);
		uv_points.push_back(uv_point);
	}

	for (int m = 0; m < num_f; m++)
	{
		obj_face* cur = face_list[m];
		Face new_face;
		
		for (int n = 0; n < cur->vertex_count; n++)
		{
			new_face.vertices.push_back(vertices[cur->vertex_index[n]]);
			new_face.normals.push_back(normals[cur->normal_index[n]]);
			new_face.uvs.push_back(uv_points[cur->texture_index[n]]);
		}

		faces.push_back(new_face);
	}

	Geometry* p = NULL;
	map<string, Geometry*>::const_iterator parentFound = objectMap.find(parent_id);
	if (parentFound != objectMap.end()) {
		p = parentFound->second;
	}

	Geometry* geo = new Geometry(id, vertices, normals, uv_points, faces, p);
	objectMap.insert(make_pair(id, geo));
	objects.push_back(geo);

	if (collidable)
	{
		collidables.push_back(geo);
	}

	// Is there a reason the loader isn't deleted? I tried uncommenting this, and nothing appeared to die a violent death.
	//  -Tyler
	//delete loader;

	cout << "DONE." << endl;

	return geo;
}

void ObjectMan::LoadTexture(string id, string name)
{
	string tex = MODELSPATH + name + TEXEXT;
	cout << "\t" << tex << "...";
	GLuint texId;

	if (TextureMan::GetInstance()->Get(id) != 0)
	{
		cout << "Previously loaded." << endl;
		return;
	}

	bool success = NeHeLoadBitmap(LPTSTR(tex.c_str()), texId);
	if(!success)
	{
		cout << "Unsuccessful." << endl;
		return;
	}
	TextureMan::GetInstance()->Add(id, texId);
	cout << "DONE." << endl;
}

Geometry* ObjectMan::getId(string id) {
	map<string, Geometry*>::iterator itr = objectMap.find(id);
	if (itr != objectMap.end()) {
		return itr->second;
	} else {
		return NULL;
	}
}

vector<Geometry*> ObjectMan::getObjects() {
	return objects;
}

vector<Geometry*> ObjectMan::getCollidables() {
	return collidables;
}