//	Class:		Geometry
//	Updated:	11/15/12
//	Project:	CS 455 Game - Racer
//
//	Representative of a Non-Deformable 3D object. Generally a container class
//	with built-in space for dynamic transforms (physics simulation, user input,
//	etc.)  Color, Texture, and other data should be stored outside this class.
//	This class is built generally, but particularly friendly to openGL.

#include "../inc/Geometry.h"
#include "../inc/TextureMan.h"

//Only for windows
#ifdef WIN32
#include <Windows.h>
#include <MMSystem.h>
#endif // WIN32

// For all operating systems
#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>

Geometry::Geometry()
	: scale(Matrix4::identityM()), rotate(Matrix4::identityM()), translate(Matrix4::identityM())
{

}

Geometry::Geometry(const Geometry& other)
	: id(other.id), vertices(other.vertices), normals(other.normals), uvs(other.uvs), faces(other.faces), point_groups(other.point_groups), scale(other.scale), rotate(other.rotate), translate(other.translate), dynamic_transforms(other.dynamic_transforms), parent(other.parent)
{

}

Geometry::~Geometry()
{

}

Geometry::Geometry(string new_id,
		vector<Vec4> _verts,
		vector<Vec4> _norms,
		vector<Vec4> _uvs,
		vector<Face> _faces,
		Geometry* _parent) : id(new_id), 
							vertices(_verts), 
							normals(_norms), 
							uvs(_uvs), 
							faces(_faces), 
							scale(Matrix4::identityM()), 
							rotate(Matrix4::identityM()), 
							translate(Matrix4::identityM()), 
							parent(_parent)
{

}

string Geometry::GetId()
{
	return id;
}

void Geometry::Scale(float x, float y, float z)
{
	scale = Matrix4::scaleM(x, y, z) * scale;
}

void Geometry::Translate(float x, float y, float z)
{
	translate *= Matrix4::translateM(x, y, z);
}

void Geometry::Rotate(float x, float y, float z)
{
	rotate *= Matrix4::rotateAxisM('x', x) *
		Matrix4::rotateAxisM('y', y) *
		Matrix4::rotateAxisM('z', z);
}

void Geometry::GenerateTransform()
{
	// Parenting
	if (parent != NULL)
	{		
		parent->GenerateTransform();
	}

	// Translate
	glMultMatrixf(translate.get());
	// Scale
	glMultMatrixf(scale.get());
	// Rotate
	glMultMatrixf(rotate.get());
}

void Geometry::SetMesh(btTriangleIndexVertexArray* _mesh)
{
	mesh = _mesh;
}

btTriangleIndexVertexArray* Geometry::GetIndexVertexArray()
{
	return mesh;
}

void Geometry::Draw()
{
	glMatrixMode(GL_MODELVIEW);
	// Save matrix state
	glPushMatrix();

	GenerateTransform();

	GLuint texid = TextureMan::GetInstance()->Get(id);
	glBindTexture(GL_TEXTURE_2D, texid);
	glBegin(GL_TRIANGLES);

	for(int f = 0; f < faces.size(); f++)
	{
		Face face = faces[f];

		for(int v = 0; v < face.vertices.size(); v++)
		{
			Vec4 vertex = face.vertices[v];
			Vec4 normal = face.normals[v];
			Vec4 uv = face.uvs[v];

			glNormal3f(normal[0], normal[1], normal[2]);
			glTexCoord2f(uv[0],	uv[1]);
			glVertex3f(vertex[0], vertex[1], vertex[2]);
		}
	}

	glEnd();

	// Undo all transforms
	glPopMatrix();
}