//	Class:		TextureMan
//	Updated:	11/19/12
//	Project:	CS 455 Game - Racer
//
//  Handles the mappings of Geometry to Textures

#ifndef TEXTUREMAN_H
#define TEXTUREMAN_H

//Only for windows
#ifdef WIN32
#include <Windows.h>
#include <MMSystem.h>
#endif // WIN32

// For all operating systems
#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>

#include <iostream>
#include <map>
#include <string>

using namespace std;

class TextureMan
{
private:
	map<string, string> toLoad;
	map<string, GLuint> textures;
	static TextureMan* instance;
	TextureMan();

public:
	static TextureMan* GetInstance()
	{
		if (instance == NULL)
		{
			instance = new TextureMan();
		}
		return instance;
	}

	void Preload(string id, string filename);
	void Add(string id, GLuint tex_id);
	GLuint Get(string id);
};

#endif