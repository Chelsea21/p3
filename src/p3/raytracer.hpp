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

#define MAX_DEPTH 2

#include "math/color.hpp"
#include "math/random462.hpp"
#include "math/vector.hpp"

namespace _462 {

struct HitRecord;
class Scene;
class Ray;
struct Intersection;
class Raytracer
{
public:

    Raytracer();

    ~Raytracer();

    bool initialize(Scene* scene, size_t num_samples,
                    size_t width, size_t height);

    bool raytrace(unsigned char* buffer, real_t* max_time);

private:

    Color3 trace_pixel(const Scene* scene,
		       size_t x,
		       size_t y,
		       size_t width,
		       size_t height);

    Color3 trace_point(const Scene* scene, Vector3 e, Vector3 d, unsigned int depth,
    					std::vector<real_t> refractive_indices) const;

    Color3 shade(const Ray ray, const HitRecord record, unsigned int depth,
    			std::vector<real_t> refractive_indices) const;

    // the scene to trace
    Scene* scene;

    // the dimensions of the image to trace
    size_t width, height;

    // the next row to raytrace
    size_t current_row;

    unsigned int num_samples;
};

inline bool is_air(const double n, const double default_n) { return std::abs(n - default_n) < 1e-6; }

} /* _462 */

#endif /* _462_RAYTRACER_HPP_ */
