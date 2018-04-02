#pragma once

#include "sceneobject.h"

class Component
{
public:
    Component();
    ~Component();

    SceneObject *scene_object;
};