//	Class:		TextureMan
//	Updated:	11/19/12
//	Project:	CS 455 Game - Racer
//
//  Handles the mappings of Geometry to Textures

#include "../inc/TextureMan.h"

#include "../inc/main.h"

#define MODELSPATH "models/"
#define TEXEXT ".bmp"

TextureMan* TextureMan::instance = NULL;

TextureMan::TextureMan()
{

}

void TextureMan::Preload(string id, string name) {
	toLoad[id] = name;
}

void TextureMan::Add(string id, GLuint tex_id)
{
	textures[id] = tex_id;
}

GLuint TextureMan::Get(string id)
{
	if (textures.find(id) == textures.end() && toLoad.find(id) != toLoad.end()) {
		string tex = MODELSPATH + toLoad[id] + TEXEXT;
		cout << "\t" << tex << "...";
		GLuint texId;

		bool success = NeHeLoadBitmap(LPTSTR(tex.c_str()), texId);
		if(!success)
		{
			cout << "Unsuccessful." << endl;
			return -1;
		}
		TextureMan::GetInstance()->Add(id, texId);
		cout << "DONE." << endl;
	}
	return textures[id];
}