#pragma once

#include <glm/fwd.hpp>

#include <random>

// For now use Mersenne twister. In future maybe use pcg (pbrt uses pcg)
// For comparison: http://www.pcg-random.org/
class RNG
{
public:
	explicit RNG(size_t seed);
	float operator()();
private:
	std::minstd_rand m_generator;
};


// Sampler
class Sampler
{
public:
    using rng_t = RNG;

    explicit Sampler(size_t seed);

    float next_float();
    int next_int(int min, int max);
    glm::vec2 next_vec2();
    glm::vec3 next_vec3();

    // Global Illuination Compendium 35
    glm::vec3 sample_hemisphere(float& pdf);
    glm::vec3 sample_sphere_uniform(const float r, float& pdf);
    glm::vec3 sample_hemisphere(const glm::vec3& wo, float& pdf);
private:
    rng_t m_rng;
};
