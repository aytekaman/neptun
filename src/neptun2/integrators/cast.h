#pragma once

#include <neptun2/integrators/integrator.h>

#include <thread>

namespace neptun
{

class CastIntegrator final : public Integrator
{
public:
    virtual ~CastIntegrator() override final = default;
    virtual void render(const Scene& scene, Image& image, const RenderingMode& rendering_mode) override final;

    size_t m_tile_size   = 16;
    size_t m_num_threads = std::thread::hardware_concurrency();
};

} // end of namespace neptun
