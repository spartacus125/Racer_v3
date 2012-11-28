//	Class:		TextureMan
//	Updated:	11/19/12
//	Project:	CS 455 Game - Racer
//
//  Handles the mappings of Geometry to Textures

#include "../inc/TextureMan.h"

TextureMan* TextureMan::instance = NULL;

TextureMan::TextureMan()
{

}

void TextureMan::Add(string id, GLuint tex_id)
{
	textures[id] = tex_id;
}

GLuint TextureMan::Get(string id)
{
	return textures[id];
}