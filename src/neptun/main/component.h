#pragma once

#include "sceneObject.h"

class Component
{
public:
	Component();
	~Component();

	SceneObject *scene_object;
};