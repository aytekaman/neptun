#include "Mesh.h"

#include <iostream>
#include <fstream>
#include <vector>

Mesh::Mesh()
{
	faceCount = 0;
}

//Mesh::Mesh(const Mesh & mesh)
//{
//}

Mesh::~Mesh()
{
}

void Mesh::CenterPivot()
{
	glm::vec3 center(0, 0, 0);

	float r = 1.0f / vertexCount;

	for (int i = 0; i < vertexCount; i++)
		center += vertices[i] * r;

	for (int i = 0; i < vertexCount; i++)
		vertices[i] -= center;

	min -= center;
	max -= center;
}