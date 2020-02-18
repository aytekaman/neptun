#pragma once

namespace neptun
{
class Scene;
class Image;

enum class RenderingMode
{
	DEFAULT,
	DEPTH,
	NORMAL,
	VISIBILITY,
	// Traversal Cost, etc
};

class Integrator
{
public:
	virtual ~Integrator() = default;
	virtual void render(const Scene& scene, Image& image, const RenderingMode& rendering_mode) = 0;
};

} // end of namespace neptun