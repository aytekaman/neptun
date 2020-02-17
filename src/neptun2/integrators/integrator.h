#pragma once

namespace neptun
{
class Scene;

class Integrator
{
public:
	virtual ~Integrator() = default;
	virtual void render(Scene& scene) = 0;
};

} // end of namespace neptun