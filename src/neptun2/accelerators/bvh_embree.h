#pragma once

#include <neptun2/accelerators/ray.h>
#include <neptun2/accelerators/accelerator.h>

#ifdef _WIN32
#define EMBREE_STATIC_LIB
#endif
#include "embree3/rtcore.h"

#include <vector>

namespace neptun
{

class BvhEmbree final : public Accelerator
{
public:
	~BvhEmbree() override = default;
	virtual bool build(const Triangle* primitives, size_t primitive_count) override final;
	virtual void intersect1(RayHit& ray_hit) const override final;
	virtual const char* name() const override final;
	virtual size_t get_size_in_bytes() const override final;

private:
	RTCScene m_rtc_scene;
	std::vector<size_t> m_geometry_ids;
	std::vector<size_t> m_primitive_ids;
};

} // end of namespace neptun