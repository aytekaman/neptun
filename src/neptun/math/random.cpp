#include "random.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// RNG
RNG::RNG(size_t seed) : m_generator(seed)
{
	
}

float RNG::operator()()
{
	return float(m_generator() - m_generator.min()) / (m_generator.max() - m_generator.min());
}

// SAMPLER
Sampler::Sampler(size_t seed) : m_rng(seed)
{

}

float Sampler::next_float()
{
    return m_rng();
}

int Sampler::next_int(int min, int max)
{
    return int(next_float() * (max - min) + min);
}

glm::vec2 Sampler::next_vec2()
{
    return glm::vec2(next_float(), next_float());
}

glm::vec3 Sampler::next_vec3()
{
    return glm::vec3(next_float(), next_float(), next_float());
}

// Global Illuination Compendium 35
glm::vec3 Sampler::sample_hemisphere(float& pdf)
{
    // random numbers in [0,1]
    const float r1 = next_float(), r2 = next_float();
    const float omega = glm::two_pi<float>() * r1;
    const float r = glm::sqrt(r2);

    glm::vec3 p;
    p.x = glm::cos(omega) * r;
    p.y = glm::sin(omega) * r;
    p.z = glm::sqrt(glm::max(1 - r2, 0.f));

    pdf = std::abs(p.z) * glm::one_over_pi<float>();

    return p;
}

glm::vec3 Sampler::sample_sphere_uniform(const float r, float& pdf)
{
    pdf = 1.0 * glm::four_over_pi<float>() / (r * r);

    const float r1 = next_float();
    const float r2 = next_float();

    const float phi = glm::two_pi<float>() * r1;
    const float s = glm::sqrt(glm::max(r2 * (1.f - r2), 0.f));
    return glm::vec3(
        2 * r * glm::cos(phi) * s,
        2 * r * glm::sin(phi) * s,
        r * (1 - 2 * r2));
}

glm::vec3 Sampler::sample_hemisphere(const glm::vec3& wo, float& pdf)
{
    constexpr float reflection_prob = 0.8f;

    if (next_float() < reflection_prob)
    {
        const glm::vec3 ref(-wo.x, -wo.y, wo.z);

        float p = 0;
        glm::vec3 d;
        do
        {
            d = sample_hemisphere(p);
            p = next_float();
        } while (p > ((glm::dot(d, wo) + 1) * 0.5f));

        pdf *= reflection_prob * ((glm::dot(d, wo) + 1) * 0.5f);
        return ref;
    }
    else
    {
        const glm::vec3 d = sample_hemisphere(pdf);
        pdf *= (1.0f - reflection_prob);
        return d;
    }
}
