#pragma once

#include <neptun2/accelerators/ray.h>
#include <neptun2/accelerators/accelerator.h>

#include <vector>

namespace neptun
{

class NoAccelerator final : public Accelerator
{
public:
	virtual bool build(const Triangle* primitives, size_t primitive_count) override final;
	virtual void intersect1(RayHit& ray_hit) const override final;
	virtual void intersect1_stats(RayHit& ray_hit, Stats& stats) const override final;
	virtual const char* name() const override final;
	virtual size_t get_size_in_bytes() const override final;

private:
	std::vector<Triangle> m_triangles;
};

} // end of namespace neptun