#pragma once

#include <string>

class Mesh;
class SceneObject;
class Texture;

class AssetImporter
{
public:
	static Mesh *ImportMesh(const char *file_name);
	static Texture *ImportTexture(const char *file_name);

	static SceneObject *CreateFromObj(const char *file_name);

	static std::string assets_folder_path;
};