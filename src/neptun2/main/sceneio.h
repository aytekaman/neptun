#pragma once

#include <neptun2/main/scene_object.h>
#include <neptun2/main/mesh.h>
#include <neptun2/main/material.h>
#include <neptun2/main/scene.h>

#include <tinyxml2/tinyxml2.h>

#include <cstdio>
#include <cstring>
#include <memory>

namespace neptun
{

std::unique_ptr<Mesh> load_mesh(const char* mesh_path, const char* file_name);

class SceneParser
{
public:
	SceneParser();
	bool parse(const char* file, Scene& scene);
	
	std::string m_error_msg;
	size_t m_error_code;
	int m_error_line;

private:
	// Tag parse functions
	bool parse_camera(const tinyxml2::XMLElement* camera_element, Camera& camera);
	bool parse_scene_object(const tinyxml2::XMLElement* element, Scene& scene);

	/*
	bool parse_scene_object();
	bool parse_mesh();
	bool parse_material();
	
	*/

	// Helpers
	bool error(size_t error_code, std::string error_msg, int line = 0);

	template <typename ... TArgs>
	inline bool parse_attribute(const tinyxml2::XMLElement* element, const char* attr_name, const char* pattern, TArgs ... args)
	{
		const tinyxml2::XMLAttribute* attr = element->FindAttribute(attr_name);

		if (attr == nullptr)
		{
			return error(1, std::string("Cannot found attribute \"") + attr_name + "\" in " + element->Name(), element->GetLineNum() );
		}

		if (sscanf(attr->Value(), pattern, args...) != sizeof...(args))
		{
			return error( 1, std::string("Cannot parse attribute \"") + attr_name + "\" in " + element->Name(), attr->GetLineNum() );
		}

		return true;
	}

	template <typename ... TArgs>
	inline bool parse_attribute_optional(const tinyxml2::XMLElement* element, const char* attr_name, const char* pattern, const char* optional, TArgs ... args)
	{
		const tinyxml2::XMLAttribute* attr = element->FindAttribute(attr_name);

		if (attr == nullptr)
		{
			if (sscanf(optional, pattern, args...) != sizeof...(args))
				return error(1, std::string("Invalid default value for attribute \"") + attr_name + "\"", element->GetLineNum());
			else
				return true;
		}

		if (sscanf(attr->Value(), pattern, args...) != sizeof...(args))
		{
			return error(1, std::string("Cannot parse attribute \"") + attr_name + "\" in " + element->Name(), attr->GetLineNum());
		}

		return true;
	}


	// Checks whether attribute with given name in element has given value 
	// Fails if attribute does not present
	bool has_attribute(const tinyxml2::XMLElement* element, const char* attr_name, const char* attr_value, bool& result);
};

} // end of namespace neptun