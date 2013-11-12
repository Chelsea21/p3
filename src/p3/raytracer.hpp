/**
 * @file raytacer.hpp
 * @brief Raytracer class
 *
 * Implement these functions for project 3.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#ifndef _462_RAYTRACER_HPP_
#define _462_RAYTRACER_HPP_

#define MAX_DEPTH 5
#define PHOTON_DEPTH 8

#include "math/color.hpp"
#include "math/random462.hpp"
#include "math/vector.hpp"
#include "scene/scene.hpp"

namespace _462 {

struct HitRecord;
class Scene;
class Ray;
struct Intersection;

struct TracingResult {
	bool is_photon;

	Color3 color;
	Photon photon;
};

class Raytracer
{
public:

    Raytracer();

    ~Raytracer();

    bool initialize(Scene* scene, size_t num_samples,
                    size_t width, size_t height);

    bool raytrace(unsigned char* buffer, real_t* max_time);
    void emit_photons() const;

private:
    Color3 trace_pixel(const Scene* scene,
		       size_t x,
		       size_t y,
		       size_t width,
		       size_t height);

    // Traces a point using the given eye ray.
    void trace_point(const Ray ray, unsigned int depth,
    					std::vector<real_t> refractive_indices, TracingResult& result) const;

    // Shades a hit point using the given eye ray.
    void shade(const Ray ray, const HitRecord record, unsigned int depth,
    			std::vector<real_t> refractive_indices, TracingResult& result) const;

    real_t shade_diffuse(const Ray ray, const HitRecord record, unsigned int depth,
    		std::vector<real_t> refractive_indices, TracingResult& result) const;

    Color3 shade_photon_map(const Vector3 hit_position) const;

    // the scene to trace
    Scene* scene;

    // the dimensions of the image to trace
    size_t width, height;

    // the next row to raytrace
    size_t current_row;

    unsigned int num_samples;

    bool photon_emission_flag;
};

// Checks whether the refractive index n is same with the refractive index default_n.
inline bool is_same_material(const double n, const double default_n) { return std::abs(n - default_n) < 1e-6; }

} /* _462 */

#endif /* _462_RAYTRACER_HPP_ */
