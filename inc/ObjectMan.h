//	Class:		ObjectMan
//	Updated:	11/18/12
//	Project:	CS 455 Game - Racer
//
//	Acts as manager for all objects within the game.

#ifndef OBJECTMAN_H
#define OBJECTMAN_H

#define CAR "car"
#define TRANSLATE_SCALE 0.25

#include <iostream>
#include <vector>
#include <map>

#include "Geometry.h"
#include "InputMan.h"

using namespace std;

class ObjectMan
{
public:
	static ObjectMan* GetInstance();
	static Geometry* GetId(string id);
	static vector<Geometry*> GetObjects();
	static vector<Geometry*> GetCollidables();
	static void UpdateObjectsWithInput();

private:
	map<string, Geometry*> objectMap;
	vector<Geometry*> objects;
	vector<Geometry*> collidables;

	static ObjectMan* instance;

	ObjectMan();
	~ObjectMan();
	void LoadObjects();
	Geometry* LoadObject(string id,
		string name,
		bool collidable,
		string parent_id);
	void LoadTexture(string id, string name);

	Geometry* getId(string id);
	vector<Geometry*> getObjects();
	vector<Geometry*> getCollidables();
};

#endif